# ESP32-LoRa

Different Firmwares to test/play around with LoRa communication

## Hardware

### LilyGo LoRa32

- [Product Page](https://github.com/LilyGO/TTGO-LORA32/tree/LilyGO-V1.3-868)

### Heltec Wireless Paper

- [Product Page](https://heltec.org/project/wireless-paper/)
- [Schematic](https://resource.heltec.cn/download/Wireless_Paper/Wireless_Paper_V0.4_Schematic_Diagram.pdf)
- [Product Page SX1262](https://www.semtech.com/products/wireless-rf/lora-connect/sx1262)

## Software in this Repo

### Arduino/WirelessPaper/LoRaSender

Heltecs Arduino-SW to demonstate cyclic TX. Mostly used to test the hardware.

### LoRa.Display

The ESP32 SW for the Heltec Wireless Paper. Currently WORK IN PROGRESS

### Simple.RXTX

A ESP32 SW for the LilyGo to demonstrate simple RX and TX

## Future Use Case

As soon as both modules work good enough with the ESP32 toolchain I want to create a simple EInk-Over-LoRa system:

- A master device (the LilyGo LoRA32) connects to a MQTT-Broker using WiFi and to multiple LoRa slaves
- Multiple LoRa slaves with a E-Ink Display to display stuff

![System](doc/system.svg)

## Software

### Software used

- [Espressif ESP-IDF v5.2.2](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html)
- In the Master: [nopnop2002s LoRa Lib](https://github.com/nopnop2002/esp-idf-sx127x.git)
- In the Slaves: [Heltec ESP32 Lib](https://github.com/HelTecAutomation/Heltec_ESP32) and for additional Info [Semtec SX126x Driver](https://github.com/Lora-net/sx126x_driver/tree/v2.3.2)

### Device Discovery - On Start

 Slaves send out a Lifesign Message on power-up

### Device Discovery - While running

1. The master sends a lifesign-request in fixed intervals
2. Slaves wait for a random time when the request is received and then transmit a lifesign message

![Discovery](doc/discovery.svg)

## TODO

- [ ] Basic Master OS
- [ ] Basic Slave OS
- [ ] Master: Generic TX for testing
- [ ] Master: Cyclic lifesign request
- [ ] Master: Detections of lifesigns
- [ ] Master: WiFi
- [ ] Master: MQTT connectivity
- [ ] Master: TX of display data
- [ ] Slave: Generic RX for testing
- [ ] Slave: Lifesign on start
- [ ] Slave: Lifesign on request
- [ ] Slave: RX of display data
- [ ] Slave: Display driver
