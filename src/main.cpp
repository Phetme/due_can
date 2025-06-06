// -----------------------------------------------------------------------------
//  Arduino Due + TJA1050 Transceiver  |  ZLAC8015D Servo Driver (CANopen)
//   Full‑featured port of your original ESP32 speed‑control sketch
//   – uses "due_can" library (https://github.com/collin80/due_can)
// -----------------------------------------------------------------------------
//  * Profile‑Velocity mode state‑machine (init → enable → run)
//  * Non‑blocking CAN (polling), SDO 1/2/4‑byte helper functions
//  * Demonstration sends L = ‑100 RPM, R = 100 RPM every 1 s when ready
//
//  ► Hardware
//      Arduino Due CAN0  ↔  TJA1050 / MCP2551
//          CAN0_TX  ==>  transceiver TXD   (Arduino pin 3)
//          CAN0_RX  ==>  transceiver RXD   (Arduino pin 4)
//          5 V + 120 Ω termination at each end of the bus
// -----------------------------------------------------------------------------
#include "Arduino.h"
#include "variant.h"
#include <due_can.h>
static const uint32_t BASE_ID = 0x10;     // ปรับได้ตามต้องการ
static const size_t   MAX_BUF = 64;       // ยาวสุดที่รองรับต่อข้อความ

void setup() {
  Serial.begin(115200);
  while (!Serial);                        // รอ Serial เปิด
  Can0.begin(CAN_BPS_500K);               // 500 kbit/s
  Serial.println("Type text then <Enter> to send over CAN");
}
/* ส่งข้อมูลยาว len ไบต์ ออก CAN 8-ไบต์/เฟรม */
void sendCAN(const uint8_t *data, size_t len) {
  for (size_t off = 0; off < len; off += 8) {
    CAN_FRAME tx;
    tx.id       = BASE_ID;
    tx.extended = 0;
    // กำหนด DLC (Data Length Code) ขึ้นอยู่กับว่า “ส่วนที่เหลือ” ยาวเท่าไร - ถ้าเหลือ ≥ 8 ไบต์ → ใส่ 8 - ถ้าเหลือน้อยกว่า 8 → ใส่จำนวนที่เหลือจริง
    tx.length   = (len - off >= 8) ? 8 : (len - off);

    /* คัดไบต์ลงเฟรม */
    for (uint8_t i = 0; i < tx.length; i++)
      tx.data.bytes[i] = data[off + i];

    if (Can0.sendFrame(tx))
      Serial.println(F("CAN ▶ sent"));
    else
      Serial.println(F("CAN ▶ queue full"));
  }
}

void loop() {
  static char  lineBuf[MAX_BUF];
  static size_t idx = 0;

  /* 1) เก็บอักษรจาก Serial ลง buffer */
  while (Serial.available()) {
    char c = Serial.read();
    Serial.println(c);
    if (c == '\r') continue;             // ข้าม CR (สำหรับบาง terminal)

    if (c == '\n') {                     // เจอ Enter → ส่ง CAN
      if (idx > 0) {
        sendCAN((uint8_t *)lineBuf, idx);
        idx = 0;                         // เคลียร์ buffer
      }
    } else if (idx < MAX_BUF) {
      lineBuf[idx++] = c;                // เก็บตัวอักษร
    }
  }
}
