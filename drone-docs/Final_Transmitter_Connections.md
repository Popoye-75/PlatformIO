# Final Transmitter Connections

## NRF24L01

  NRF24 Pin   Arduino Nano
  ----------- ---------------
  VCC         3.3V
  GND         GND
  CE          D7
  CSN         D8
  SCK         D13
  MOSI        D11
  MISO        D12
  IRQ         Not Connected

> Add a 10uF capacitor across VCC and GND.

## ST7789 TFT

  TFT Pin    Arduino Nano
  ---------- ---------------------------------------------------------
  VCC        Module VCC (5V if breakout supports it, otherwise 3.3V)
  GND        GND
  SCL/SCK    D13
  SDA/MOSI   D11
  CS         D10
  DC         D9
  RST        D6
  BL         VCC

## Joysticks

-   A0: Left X
-   A1: Left Y
-   A2: Right X
-   A3: Right Y

## Battery Sense

-   A4

## Buttons

-   D2: MODE
-   D3: FUNCTION

## Rotary Encoder

-   D4: CLK
-   D5: DT
-   A5: SW

## Shared SPI

-   D11 MOSI
-   D12 MISO
-   D13 SCK
