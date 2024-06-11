
# Different ESP32 projects to play around with LoRa

I mostly use two hardwares I hat flying around:

- [LilyGo Lora32 v1.3](https://github.com/LilyGO/TTGO-LORA32/tree/LilyGO-V1.3-868)
- [Heltec Wireless Paper](https://heltec.org/project/wireless-paper/)

Software used:

- [Espressif ESP-IDF v5.2.2](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html)
- [nopnop2002s LoRa Lib](https://github.com/nopnop2002/esp-idf-sx127x.git)

## Simple Receiver

A first test for the LilyGo module. Simply receives stuff and prints out some info on the console and in the LCD. Also used for testing the pinning because the LilyGo pinout is wrong. But the schematic is correct at least
