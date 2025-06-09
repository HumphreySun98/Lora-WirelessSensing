#include <RadioLib.h>
#include "LoRaBoards.h"

#if !defined(USING_SX1280) && !defined(USING_SX1280PA)
#define USING_SX1280
#endif

SX1280 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN,
                          RADIO_RST_PIN, RADIO_BUSY_PIN);

void setup() {
  Serial.begin(115200);               // ← 新增：打开串口
  pinMode(BOARD_LED, OUTPUT);         // 若 LoRaBoards.h 已定义 BOARD_LED 可省

  radio.begin(2445.0, 203.125, 10, 6, 8, 13, 4);
#if defined(RADIO_RX_PIN) && defined(RADIO_TX_PIN)
  radio.setRfSwitchPins(RADIO_RX_PIN, RADIO_TX_PIN);
#endif
  Serial.println("Beacon started");
}

void loop() {
  static uint32_t cnt = 0;
  radio.transmit((uint8_t *)&cnt, 4);
  Serial.printf("TX %lu\n", cnt);     // ← 新增：打印计数
  digitalWrite(BOARD_LED, !digitalRead(BOARD_LED));  // ← LED 翻转
  cnt++;
  delay(10);
}
