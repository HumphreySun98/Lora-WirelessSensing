#include "LoRaBoards.h"

void setup() {
  Serial.begin(115200);
  pinMode(RADIO_RST_PIN,  OUTPUT);
  pinMode(RADIO_TCXO_ENABLE, OUTPUT);
  pinMode(RADIO_BUSY_PIN, INPUT);

  // 1) 给 TCXO 供电
  Serial.println("TCXO HIGH");
  digitalWrite(RADIO_TCXO_ENABLE, HIGH);
  delay(5);

  // 2) 打印 BUSY 电平 (上电后)
  Serial.printf("BUSY after TCXO: %d\n", digitalRead(RADIO_BUSY_PIN));

  // 3) 硬复位拉低 2 ms
  Serial.println("RST LOW 2 ms");
  digitalWrite(RADIO_RST_PIN, LOW);
  delay(2);
  digitalWrite(RADIO_RST_PIN, HIGH);
  delay(5);

  // 4) 再读 BUSY
  Serial.printf("BUSY after RST : %d\n", digitalRead(RADIO_BUSY_PIN));
}

void loop() { delay(1000); }
