# hzgl-lora-communicator

["hzgl-lora-communicator"](https://github.com/HouzuoGuo/hzgl-lora-communicator)
is an open source software for a battery-powered two-way messaging device. It
uses [The Things Network Community Edition](https://www.thethingsindustries.com/docs/getting-started/ttn/)
(LoRaWAN) for bi-directional data connectivity.

## Features

- Free to use with good coverage in nearly all major cities [around the world](https://www.thethingsnetwork.org/map).
  * Note: as of December 2021, the software is configured to work in Europe.
    This may be improved in the future.
- Send text messages up to 100 characters in length.
- Receive text messages up to 50 characters in length.
- Display and transmit GPS location.
- Display and transmit environment (temperature, humidity, pressure) readings.
- Fox-hunt nearby 2.4GHz WiFi and Bluetooth LE transmitters.

## Supported hardware

As of December 2021, the software exclusively targets the TTGO T-Beam board
revision v1.1. In addition, the software is configured to work with European
cities by using the `EU868` frequency plan. These may change in the future.

"TTGO T-Beam" is an IoT development board manufactured by Chinese company
["LILYGO"](https://twitter.com/lilygo9). The board features:

- An ESP32 microcontroller with built-in 2.4GHz WiFi and Bluetooth LE
  transceivers.
- A LoRa transceiver.
- A GPS receiver and its ceramic antenna.
- A battery holder for 3.7V 18650 cell.
- A 0.96 inch OLED.
  * The display is often sold as an add-on.
- A BME280 environment (temperature, humidity, and pressure) sensor.
  * The software will continue to function even without this sensor.
  * Be aware that some Chinese merchants deceive buyers by shipping them BMP280
    sensors. The software does not support BMP280.

<img src="https://github.com/HouzuoGuo/hzgl-lora-communicator/raw/master/media/system-monitor.png" />

## Get started

Follow wiki article [Get started](https://github.com/HouzuoGuo/hzgl-lora-communicator/wiki/Get-started)
to create a new device on The Things Network console, and write the firmware
onto your TTGO T-Beam.

Then check out wiki article [Usage](https://github.com/HouzuoGuo/hzgl-lora-communicator/wiki/Usage).

## Copyright

Copyright (C) 2021 Google Inc. All rights reserved.

This program is free software subject to the terms of GNU Public License, v 3.0.
You may find the license text in the [LICENSE file](https://github.com/HouzuoGuo/hzgl-lora-communicator/blob/master/LICENSE).

This is not an officially supported Google product.
