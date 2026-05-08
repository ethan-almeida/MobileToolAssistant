#include <SPI.h>

void setup() {
Serial.begin(115200);

pinMode(D4, OUTPUT); // LED pin
pinMode(D8, OUTPUT); // CS pin
digitalWrite(D8, HIGH); // idle high

SPI.begin(); // default: SCK=14, MISO=12, MOSI=13 (internal)


}

void loop() {

uint8_t tx[5] = {0x00,0x00,0x00,0x00,0x00};
uint8_t rx[5] = {0};

digitalWrite(D4, LOW); // LED ON
delay(200); // visible ON time

SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));

digitalWrite(D8, LOW);

for (int i = 0; i < 5; i++) {
rx[i] = SPI.transfer(tx[i]);
}

digitalWrite(D8, HIGH);
SPI.endTransaction();

digitalWrite(D4, HIGH); // LED OFF
uint32_t dev_id = rx[1] | (rx[2] << 8) | (rx[3] << 16) | (rx[4] << 24);

Serial.print("Device ID: 0x");
Serial.println(dev_id, HEX);
delay(800);

}