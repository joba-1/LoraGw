# Lora Gateway with ESP8266

Receive messages from lora nodes and post them on syslog and influx

## Hardware

* RFM95W 868MHz Lora chip
* Wemos Mini D1 Pro ESP8266
* Receiving indicator LED+ at ESP-D1
* WLAN builtin LED- at ESP-D4 

| ESP  | RFM  | Remarks  |
|------|------|----------|
| GND  | GND  |          |
| 3V3  | 3.3V |          |
| D0   | DIO5 | no IRQ   |
| D3   | NSS  | pullup   |
| D5   | SCK  | hardware |
| D6   | MISO | hardware |
| D7   | MOSI | hardware |
| D8   | DIO0 | pulldown |
