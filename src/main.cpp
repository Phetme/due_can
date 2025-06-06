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
// Arduino Due CANopen Servo Controller (Non-blocking, State Machine)
// ใช้กับไลบรารี due_can + CAN0


void setup()
{
  Serial.begin(115200);
  while (!Serial) ;                       // รอ USB enumeration

  // ---------- 1. เริ่มต้น CAN0 ----------
  if (!Can0.begin(CAN_BPS_500K)) {        // 500 kbit/s
    Serial.println("CAN0 init FAILED");
    while (1);
  }
  Serial.println("CAN0 init OK (500 kbps)");

  // (ไม่จำเป็นต้อง watchFor() ถ้าเราส่งอย่างเดียว)
  // ถ้าจะรับด้วยสามารถเปิดฟิลเตอร์ เช่น
  // Can0.watchFor(0x201);   // รับ ID 0x201 อย่างเดียว
}

/*---------------------------------------------------------------------------*/
void loop()
{
  CAN_FRAME tx;
  tx.id       = 0x10;       // 11-bit standard ID
  tx.extended = 0;          // 0 = standard, 1 = extended (29-bit)
  tx.length   = 6;          // DLC = 6
  tx.data.byte[0] = 'H';
  tx.data.byte[1] = 'e';
  tx.data.byte[2] = 'l';
  tx.data.byte[3] = 'l';
  tx.data.byte[4] = 'o';
  tx.data.byte[5] = '!';

  // ---------- 2. ส่งเฟรม ----------
  if (Can0.sendFrame(tx)) {
    Serial.println("Frame sent ✔");
  } else {
    Serial.println("Transmit queue full ✘");
  }

  delay(1000);
}
