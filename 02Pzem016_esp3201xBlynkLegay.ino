//================== Modbus ==================//
#include <ModbusMaster.h>
#include <util/crc16.h>

//================== Blynk Legacy ==================//
#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// --------- ใส่ค่าของคุณ --------- //
char auth[] = "BLYNK_AUTH_TOKEN";
char ssid[] = "WIFI_SSID";
char pass[] = "WIFI_PASS";

// ✅ Custom Blynk Server
char blynk_server[] = "YOUR_SERVER_DOMAIN_OR_IP";   // เช่น "192.168.1.10" หรือ "blynk.yourdomain.com"
uint16_t blynk_port = 8080;                         // ค่า default Legacy มักเป็น 8080 (non-SSL)

// --------- PIN (ESP32 Serial2) --------- //
#define RX2 26
#define TX2 27

ModbusMaster node;
static uint8_t pzemSlaveAddr = 0x01;

// --------- Timing --------- //
const uint32_t READ_INTERVAL_MS = 1000;
uint32_t lastReadMs = 0;

//============== Reset Energy (PZEM custom cmd 0x42) ============//
void resetEnergy(uint8_t slaveAddr) {
  uint16_t crc = 0xFFFF;
  const uint8_t resetCommand = 0x42;

  while (Serial2.available()) Serial2.read();

  crc = crc16_update(crc, slaveAddr);
  crc = crc16_update(crc, resetCommand);

  Serial.println("Resetting Energy...");

  Serial2.write(slaveAddr);
  Serial2.write(resetCommand);
  Serial2.write(lowByte(crc));
  Serial2.write(highByte(crc));
  Serial2.flush();

  uint32_t t0 = millis();
  while ((millis() - t0) < 200) {
    while (Serial2.available()) {
      uint8_t b = Serial2.read();
      Serial.print(b, HEX);
      Serial.print(" ");
    }
  }
  Serial.println();
}

//============== Setup Function ============//
void setup() {
  Serial.begin(9600);

  Serial2.begin(9600, SERIAL_8N1, RX2, TX2);
  node.begin(pzemSlaveAddr, Serial2);

  // ✅ ต่อ Blynk Legacy แบบ Custom Server
  // รูปแบบ: Blynk.begin(auth, ssid, pass, server, port);
  Blynk.begin(auth, ssid, pass, blynk_server, blynk_port);

  Serial.println("PZEM Modbus Ready + Blynk Custom Server Connected");
}

//============== Loop Function ============//
void loop() {
  Blynk.run();

  if (millis() - lastReadMs < READ_INTERVAL_MS) return;
  lastReadMs = millis();

  uint8_t result = node.readInputRegisters(0x0000, 9);

  if (result == node.ku8MBSuccess) {
    uint32_t temp32;

    float voltage = node.getResponseBuffer(0x0000) / 10.0;

    temp32 = ((uint32_t)node.getResponseBuffer(0x0002) << 16) | node.getResponseBuffer(0x0001);
    float current = temp32 / 1000.0;

    temp32 = ((uint32_t)node.getResponseBuffer(0x0004) << 16) | node.getResponseBuffer(0x0003);
    float power = temp32 / 10.0;

    temp32 = ((uint32_t)node.getResponseBuffer(0x0006) << 16) | node.getResponseBuffer(0x0005);
    float energy_wh = (float)temp32;

    float hz = node.getResponseBuffer(0x0007) / 10.0;
    float pf = node.getResponseBuffer(0x0008) / 100.0;

    // Serial
    Serial.print(voltage, 1); Serial.print(" V   ");
    Serial.print(hz, 1);      Serial.print(" Hz   ");
    Serial.print(current, 3); Serial.print(" A   ");
    Serial.print(power, 1);   Serial.print(" W   ");
    Serial.print(pf, 2);      Serial.print(" pf   ");
    Serial.print(energy_wh, 0); Serial.print(" Wh");
    Serial.println();

    // ✅ ส่งขึ้น Blynk (ปรับ V-pin ได้ตามต้องการ)
    Blynk.virtualWrite(V0, voltage);
    Blynk.virtualWrite(V1, current);
    Blynk.virtualWrite(V2, power);
    Blynk.virtualWrite(V3, energy_wh);
    Blynk.virtualWrite(V4, hz);
    Blynk.virtualWrite(V5, pf);

  } else {
    Serial.print("Modbus read failed, code: ");
    Serial.println(result);
    Blynk.virtualWrite(V10, (int)result);
  }
}
