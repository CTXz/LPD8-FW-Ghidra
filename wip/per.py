#!/usr/bin/env python3

import argparse

peripheral_map = {
    0xA0000000: "FSMC",
    0x50000000: "USB OTG FS",
    0x40028000: "Ethernet",
    0x40023000: "CRC",
    0x40022000: "Flash memory interface",
    0x40021000: "Reset and clock control (RCC)",
    0x40020000: "DMA1",
    0x40018000: "SDIO",
    0x40015400: "TIM11 timer",
    0x40015000: "TIM10 timer",
    0x40014C00: "TIM9 timer",
    0x40013C00: "ADC3",
    0x40013800: "USART1",
    0x40013400: "TIM8 timer",
    0x40013000: "SPI1",
    0x40012C00: "TIM1 timer",
    0x40012800: "ADC2",
    0x40012400: "ADC1",
    0x40012000: "GPIO Port G",
    0x40011C00: "GPIO Port F",
    0x40011800: "GPIO Port E",
    0x40011400: "GPIO Port D",
    0x40011000: "GPIO Port C",
    0x40010C00: "GPIO Port B",
    0x40010800: "GPIO Port A",
    0x40010400: "EXTI",
    0x40010000: "AFIO",
    0x4000F400: "DAC",
    0x4000F000: "Power control (PWR)",
    0x40006C00: "Backup registers (BKP)",
    0x40006400: "bxCAN1",
    0x40006800: "bxCAN2",
    0x40006000: "Shared USB/CAN SRAM",
    0x40005C00: "USB device FS registers",
    0x40005800: "I2C2",
    0x40005400: "I2C1",
    0x40005000: "UART5",
    0x40004C00: "UART4",
    0x40004800: "USART3",
    0x40004400: "USART2",
    0x40003C00: "SPI3/I2S",
    0x40003800: "SPI2/I2S",
    0x40003000: "Independent watchdog (IWDG)",
    0x40002C00: "Window watchdog (WWDG)",
    0x40002800: "RTC",
    0x40002000: "TIM14 timer",
    0x40001C00: "TIM13 timer",
    0x40001800: "TIM12 timer",
    0x40001400: "TIM7 timer",
    0x40001000: "TIM6 timer",
    0x40000C00: "TIM5 timer",
    0x40000800: "TIM4 timer",
    0x40000400: "TIM3 timer",
    0x40000000: "TIM2 timer",
}


def get_peripheral_name(address):
    for peripheral_address in peripheral_map:
        if address >= peripheral_address:
            diff = address - peripheral_address
            return (peripheral_map[peripheral_address], hex(diff))

    return None


arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("address", type=lambda x: int(x, 0))
args = arg_parser.parse_args()

per, offset = get_peripheral_name(args.address)
if per is None:
    print("Unknown peripheral")
else:
    print("Peripheral: {}, Offset: {}".format(per, offset))
