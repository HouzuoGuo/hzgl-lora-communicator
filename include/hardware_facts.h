#pragma once

#include <axp20x.h>

// There are a few sources that describe the hardware capabilities and configuration:
// - https://doc.riot-os.org/group__boards__esp32__ttgo-t-beam.html
// - https://github.com/lnlp/pinout-diagrams/blob/main/LoRa%20development%20boards/TTGO%20T-Beam%20V1.1%20Pinout.pdf
// - https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series
// - https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/master/schematic/LilyGo_TBeam_V1.1.pdf

// SPI_SCK_GPIO is described on schematic as: SPI_DEV(0):CLK - GPIO5
#define SPI_SCK_GPIO 5
// SPI_MISO_GPIO is described on schematic as: SPI_DEV(0):MISO - GPIO19
#define SPI_MISO_GPIO 19
// SPI_MOSI_GPIO is described on schematic as: SPI_DEV(0):MOSI - GPIO27
#define SPI_MOSI_GPIO 27
// SPI_NSS_GPIO is described on schematic as: SPI_DEV(0):CS0 - GPIO18
#define SPI_NSS_GPIO 18
// LORA_RST_GPIO is described on schematic as (according to seller's pin diagram): LORA - RST - 23.
#define LORA_RST_GPIO 23
// LORA_DIO0_GPIO is described on schematic as (according to seller's pin diagram):  LORA - DIO - 26.
#define LORA_DIO0_GPIO 26
// LORA_DIO1_GPIO is a bit of a mystery. I am unsure how this "ADC 5 GPIO 33" pin is relevant to LoRa.
#define LORA_DIO1_GPIO 33
// LORA_DIO2_GPIO is a bit of mystery. I am unsure how this "ADC 4 GPIO 32" pin is relevant to LoRa.
#define LORA_DIO2_GPIO 32
// GPS_SERIAL_RX is described on schematic as: UART_DEV(1):RxD GPIO12 GPS
// Be aware of the curious remark made by seller's pin diagram that noted "ESP32(TX)".
#define GPS_SERIAL_RX 12
// GPS_SERIAL_RX is described on schematic as: UART_DEV(1):TxD GPIO34 GPS
// Be aware of the curious remark made by seller's pin diagram that noted "ESP32(RX)".
#define GPS_SERIAL_TX 34

// I2C_FREQUENCY_HZ is slower than the default frequency (700kHz) to cater for the CPU frequency used during power-saving configuration.
#define I2C_FREQUENCY_HZ 400000

// I2C_SCL is described on schematic as: I2C_DEV(0):SCL - GPIO22
#define I2C_SCL 22
// I2C_SDA is described on schematic as: I2C_DEV(0):SDA - GPIO21
#define I2C_SDA 21

// OLED_I2C_ADDR is the I2C address of the on-board OLED hardware.
#define OLED_I2C_ADDR 0x3c
// OLED_MAX_LINE_LEN is the maximum number of characters that fit into a single line when using font ArialMT_Plain_10.
#define OLED_MAX_LINE_LEN 23
// OLED_MAX_NUM_LINES is the maximum number of lines that fit on the display using when font ArialMT_Plain_10.
#define OLED_MAX_NUM_LINES 6
// OLED_FONT_HEIGHT_PX is the height (in pixels) of characters displayed when using font ArialMT_Plain_10.
#define OLED_FONT_HEIGHT_PX 10

// BME280_I2C_ADDR is the I2C address of the on-board BME280 break-out board.
#define BME280_I2C_ADDR 0x76

// GENERIC_PURPOSE_BUTTON is the GPIO pin number of the only generic purpose programmable button on the board.
#define GENERIC_PURPOSE_BUTTON 38

// TEMP_OFFSET_CELCIUS_BATT is the temperature reading offset to be added (or substracted when negative)
// when the microcontroller board is powered by battery alone.
// The BME280 sensor is mounted on the battery-holder side of the board parallel to the battery holder.
#define TEMP_OFFSET_CELCIUS_BATT -2.0716

// TEMP_OFFSET_CELCIUS_USB is the temperature reading offset to be added (or substracted when negative)
// when the microcontroller board is powered by USB alone and not charging.
// The BME280 sensor is mounted on the battery-holder side of the board parallel to the battery holder.
#define TEMP_OFFSET_CELCIUS_USB -2.9487