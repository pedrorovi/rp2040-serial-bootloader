/**
 * Copyright (c) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include <string.h>

#include "RP2040.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/resets.h"
#include "hardware/structs/dma.h"
#include "hardware/structs/watchdog.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "pico/time.h"

#ifdef DEBUG
#include <stdio.h>
#include "pico/stdio_usb.h"
#define DBG_PRINTF_INIT() stdio_usb_init()
#define DBG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DBG_PRINTF_INIT() \
    {}
#define DBG_PRINTF(...) \
    {}
#endif

// The bootloader can be entered in three ways:
//  - Watchdog scratch[5] == BOOTLOADER_ENTRY_MAGIC && scratch[6] == ~BOOTLOADER_ENTRY_MAGIC
//  - No valid image header
#define BOOTLOADER_ENTRY_MAGIC 0xb105f00d

#define UART_TX_PIN 8
#define UART_RX_PIN 9
#define UART_BAUD 115200

#define IMAGE_HEADER_OFFSET_1 (12 * 1024)
#define IMAGE_HEADER_OFFSET_2 (1024 * 1024)

#define WRITE_ADDR_MIN (XIP_BASE + IMAGE_HEADER_OFFSET_1 + FLASH_SECTOR_SIZE)
#define ERASE_ADDR_MIN (XIP_BASE + IMAGE_HEADER_OFFSET_1)
#define FLASH_ADDR_MAX (XIP_BASE + PICO_FLASH_SIZE_BYTES)

static void disable_interrupts(void) {
    SysTick->CTRL &= ~1;

    NVIC->ICER[0] = 0xFFFFFFFF;
    NVIC->ICPR[0] = 0xFFFFFFFF;
}

static void reset_peripherals(void) {
    reset_block(~(RESETS_RESET_IO_QSPI_BITS | RESETS_RESET_PADS_QSPI_BITS | RESETS_RESET_SYSCFG_BITS |
                  RESETS_RESET_PLL_SYS_BITS));
}

static void jump_to_vtor(uint32_t vtor, uint32_t reset_vector) {
    // Derived from the Leaf Labs Cortex-M3 bootloader.
    // Copyright (c) 2010 LeafLabs LLC.
    // Modified 2021 Brian Starkey <stark3y@gmail.com>
    // Originally under The MIT License

    SCB->VTOR = (volatile uint32_t)(vtor);

    asm volatile("msr msp, %0" ::"g"(*(volatile uint32_t*)vtor));
    asm volatile("bx %0" ::"r"(reset_vector));
}

// ptr must be 4-byte aligned and len must be a multiple of 4
static uint32_t calc_crc32(void* ptr, uint32_t len) {
    uint32_t dummy_dest, crc;

    int channel = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_sniff_enable(&c, true);

    // Seed the CRC calculation
    dma_hw->sniff_data = 0xffffffff;

    // Mode 1, then bit-reverse the result gives the same result as
    // golang's IEEE802.3 implementation
    dma_sniffer_enable(channel, 0x1, true);
    dma_hw->sniff_ctrl |= DMA_SNIFF_CTRL_OUT_REV_BITS;

    dma_channel_configure(channel, &c, &dummy_dest, ptr, len / 4, true);

    dma_channel_wait_for_finish_blocking(channel);

    // Read the result before resetting
    crc = dma_hw->sniff_data ^ 0xffffffff;

    dma_sniffer_disable();
    dma_channel_unclaim(channel);

    return crc;
}

struct image_header {
    uint32_t vtor;
    uint32_t size;
    uint32_t crc;
    uint8_t pad[FLASH_PAGE_SIZE - (3 * 4)];
};
static_assert(sizeof(struct image_header) == FLASH_PAGE_SIZE, "image_header must be FLASH_PAGE_SIZE bytes");

static void uint32_to_string(const uint32_t val, char* buffer) {
    // create a string representation of val in hex format with leading zeros
    // buffer must be at least 9 bytes long
    // without snprintf
    const char hex[] = "0123456789abcdef";
    buffer[0] = '0';
    buffer[1] = 'x';
    buffer[2] = hex[(val >> 28) & 0xf];
    buffer[3] = hex[(val >> 24) & 0xf];
    buffer[4] = hex[(val >> 20) & 0xf];
    buffer[5] = hex[(val >> 16) & 0xf];
    buffer[6] = hex[(val >> 12) & 0xf];
    buffer[7] = hex[(val >> 8) & 0xf];
    buffer[8] = hex[(val >> 4) & 0xf];
    buffer[9] = hex[val & 0xf];
    buffer[10] = 0;
}

static bool image_header_ok(struct image_header* hdr) {
    uint32_t* vtor = (uint32_t*)hdr->vtor;

    uint32_t calc = calc_crc32((void*)hdr->vtor, hdr->size);

    // CRC has to match
    if (calc != hdr->crc) {
        // const char format[] = "CRC mismatch calc: 0x%08x hdr->crc: 0x%08x\r\n";
        char text[32];
        // snprintf(text, 64, format, calc, hdr->crc);
        uint32_to_string(calc, text);
        uart_puts(uart1, text);
        return false;
    }

    // Stack pointer needs to be in RAM
    if (vtor[0] < SRAM_BASE) {
        uart_puts(uart1, "SP out of range\r\n");
        return false;
    }

    // Reset vector should be in the image, and thumb (bit 0 set)
    if ((vtor[1] < hdr->vtor) || (vtor[1] > hdr->vtor + hdr->size) || !(vtor[1] & 1)) {
        uart_puts(uart1, "Reset vector out of range\r\n");
        return false;
    }

    // Looks OK.
    return true;
}

static void do_reboot(bool to_bootloader) {
    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
    if (to_bootloader) {
        watchdog_hw->scratch[5] = BOOTLOADER_ENTRY_MAGIC;
        watchdog_hw->scratch[6] = ~BOOTLOADER_ENTRY_MAGIC;
    } else {
        watchdog_hw->scratch[5] = 0;
        watchdog_hw->scratch[6] = 0;
    }
    watchdog_reboot(0, 0, 0);
    while (1) {
        tight_loop_contents();
        asm("");
    }
}

static bool should_stay_in_bootloader() {
    bool wd_says_so =
        (watchdog_hw->scratch[5] == BOOTLOADER_ENTRY_MAGIC) && (watchdog_hw->scratch[6] == ~BOOTLOADER_ENTRY_MAGIC);

    return wd_says_so;
}

static void init_gpio(const uint8_t gpio) {
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_put(gpio, 1);
}

static void init_uart() {
    uart_init(uart1, UART_BAUD);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(uart1, false, false);
}

int main(void) {
    init_gpio(PICO_DEFAULT_LED_PIN);
    init_gpio(23);
    init_gpio(2);
    init_uart();
    sleep_ms(100);

    uart_puts(uart1, "Hello, World!\r\n");
    sleep_ms(100);

    uint32_t vtor = (XIP_BASE + IMAGE_HEADER_OFFSET_1);
    // uint32_t vtor = *((uint32_t*)(XIP_BASE + IMAGE_HEADER_OFFSET_1));
    char text[32];
    uart_puts(uart1, "VTOR: ");
    uint32_to_string(XIP_BASE + IMAGE_HEADER_OFFSET_1, text);
    uart_puts(uart1, text);
    uart_puts(uart1, " - ");
    uint32_to_string(vtor, text);
    uart_puts(uart1, text);
    uart_puts(uart1, "\r\n");

    // uint32_t reset_vtor = XIP_BASE + IMAGE_HEADER_OFFSET_1 + 0x04;
    uint32_t reset_vtor = *((uint32_t*)(XIP_BASE + IMAGE_HEADER_OFFSET_1 + 0x04));
    uart_puts(uart1, "Reset VTOR: ");
    uint32_to_string(XIP_BASE + IMAGE_HEADER_OFFSET_1 + 0x04, text);
    uart_puts(uart1, text);
    uart_puts(uart1, " - ");
    uint32_to_string(reset_vtor, text);
    uart_puts(uart1, text);
    uart_puts(uart1, "\r\n");
    sleep_ms(100);

    disable_interrupts();
    reset_peripherals();
    jump_to_vtor(vtor, reset_vtor);

    // In an assembly snippet . . .
    // Set VTOR register, set stack pointer, and jump to reset

    /* asm volatile(
        "mov r0, %[start]\n"
        "ldr r1, =%[vtable]\n"
        "str r0, [r1]\n"
        "ldmia r0, {r0, r1}\n"
        "msr msp, r0\n"
        "bx r1\n"
        :
        : [start] "r"(XIP_BASE + IMAGE_HEADER_OFFSET_1), [vtable] "X"(PPB_BASE + M0PLUS_VTOR_OFFSET)
        :);
    */

    uart_puts(uart1, "Power off . . .\r\n");
    gpio_put(23, 0);

    return 0;
}
