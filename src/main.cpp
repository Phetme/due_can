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

// ====== CONFIG ======
#define SERVO_NODE_ID 1
#define GUIDE_SENSOR_NODE_ID 1

#define CAN_INTERFACE Can0

// COB-IDs
#define SDO_REQUEST(node_id) (0x600 + node_id)
#define SDO_RESPONSE(node_id) (0x580 + node_id)
#define CHECK_RESPONSE(node_id) (0x700 + node_id)
#define GUIDE_SENSOR_REQUEST(node_id) (0x000 + node_id)

// Object Dictionary
#define OD_ENABLE_MODE 0x6040
#define OD_OPERATION_MODE 0x6060
#define OD_TARGET_VELOCITY 0x60FF
#define OD_ACTUAL_VELOCITY 0x606C

#define MODE_PROFILE_VELOCITY 3
#define WRITE_1_BYTE 0x2F
#define WRITE_2_BYTES 0x2B
#define WRITE_4_BYTES 0x23
#define READ_REQUEST 0x40

#define CTRL_SHUTDOWN 0x0006
#define CTRL_SWITCH_ON 0x0007
#define CTRL_ENABLE_OPERATION 0x000F

//========GUIDE SENSOR 16BIT========//

// CANBUS GUIDE SENSOR
#define CMD_GUIDE_SENSOR 0xABCD
// ADDRESS CANBUS
#define READ_GUIDE_SENSOR 0x28
// DATA CANBUS
#define DATA_GUIDE_SENSOR 0x01

//=================================//

/// @param startTime
/// @param timeoutMs
/// @return
bool isTimeout(unsigned long startTime, unsigned long timeoutMs);
// ====== STATE ======
enum InitState
{
  INIT_START,
  INIT_SET_MODE,
  INIT_ENABLE1,
  INIT_ENABLE2,
  INIT_ENABLE3,
  STATE_READY,
  STATE_ERROR,
  INIT_SENSOR_GUILD
};

InitState currentState = INIT_START;
unsigned long stateStartTime = 0;
bool waitingForResponse = false;
bool init_can = false;

// ====== STRUCT ======
struct SpeedController
{
  int32_t target_speed_rpm;
  int32_t actual_speed_rpm;
  uint16_t status_word;
  bool servo_enabled;
  InitState init_state;
  unsigned long last_speed_command;
  unsigned long last_status_read;
  unsigned long last_status_print;
  unsigned long last_init_step;
  bool pending_speed_update;
  int32_t pending_speed_value;
  bool pending_save_request;
  bool save_motor_params;
  unsigned long save_request_time;
};

SpeedController speed_ctrl = {};

struct TimeControl
{
  unsigned long time_set;
  unsigned long prve_set;
  unsigned long time_check_msg;
  unsigned long prve_check_msg;
  unsigned long time_guide_sensor;
  unsigned long prve_guide_sensor;
};

TimeControl time_control = {};

// ====== DECLARATIONS ======
void processCheckConnectWithTimeout(unsigned long timeout_ms);
void updateStateMachine();
void checkCANMessages();
bool sendGuideSensor_1Byte(uint16_t index, uint8_t address, uint8_t data);

// ====== SETUP ======
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println("Starting CAN0 at 500kbps...");
  CAN_INTERFACE.begin(CAN_BPS_500K);
  CAN_INTERFACE.watchFor();

  delay(500);
  processCheckConnectWithTimeout(10000);
  stateStartTime = millis();
}

void loop()
{
  time_control.time_set = millis();
  time_control.time_check_msg = millis();

  if (time_control.time_set - time_control.prve_set >= (1000 / 50))
  {
    if (!waitingForResponse)
    {
      updateStateMachine();
    }
    time_control.prve_set = time_control.time_set;
  }

  if (time_control.time_check_msg - time_control.prve_check_msg >= (1000 / 10))
  {
    if (waitingForResponse)
    {
      checkCANMessages();
    }
    time_control.prve_check_msg = time_control.time_check_msg;
  }
  if (time_control.time_guide_sensor - time_control.prve_guide_sensor >= (1000 / 100)){

  if (!waitingForResponse || isTimeout(stateStartTime, 1000))
    {
      if (sendGuideSensor_1Byte(CMD_GUIDE_SENSOR, READ_GUIDE_SENSOR, DATA_GUIDE_SENSOR))
      {
        waitingForResponse = true;
        stateStartTime = millis();
        currentState = STATE_READY;
      }
    }
      
    time_control.prve_guide_sensor = time_control.time_guide_sensor;

  }
}

// ====== SDO SEND ======
bool sendSDO_1Byte(uint16_t index, uint8_t subindex, uint8_t data)
{
  CAN_FRAME frame;
  frame.id = SDO_REQUEST(SERVO_NODE_ID);
  frame.length = 8;
  frame.extended = false;
  frame.data.bytes[0] = WRITE_1_BYTE;
  frame.data.bytes[1] = index & 0xFF;
  frame.data.bytes[2] = index >> 8;
  frame.data.bytes[3] = subindex;
  frame.data.bytes[4] = data;
  frame.data.bytes[5] = 0;
  frame.data.bytes[6] = 0;
  frame.data.bytes[7] = 0;
  bool sent = CAN_INTERFACE.sendFrame(frame);
  if (sent)
  {
    char buf[64];
    snprintf(buf, sizeof(buf), "[TX] SDO 1B %04X:%02X = %02X", index, subindex, data);
    Serial.println(buf);
  }
  return sent;
}

// ====== SDO SEND GUDIE SENSOR ======
bool sendGuideSensor_1Byte(uint16_t index, uint8_t address, uint8_t data)
{
  CAN_FRAME frame;
  frame.id = GUIDE_SENSOR_REQUEST(GUIDE_SENSOR_NODE_ID);
  frame.length = 8;
  frame.extended = false;
  frame.data.bytes[0] = index & 0xFF;
  frame.data.bytes[1] = index >> 8;
  frame.data.bytes[2] = 0;
  frame.data.bytes[3] = address;
  frame.data.bytes[4] = 0;
  frame.data.bytes[5] = data;
  frame.data.bytes[6] = 0;
  frame.data.bytes[7] = 0;
  bool sent = CAN_INTERFACE.sendFrame(frame);
  if (sent)
  {
    char buf[64];
    snprintf(buf, sizeof(buf), "[TX] SDO 1B %04X:%02X = %02X", index, address, data);
    Serial.println(buf);
  }
  return sent;
}

bool sendSDO_2Bytes(uint16_t index, uint8_t subindex, uint16_t data)
{
  CAN_FRAME frame;
  frame.id = SDO_REQUEST(SERVO_NODE_ID);
  frame.length = 8;
  frame.extended = false;
  frame.data.bytes[0] = WRITE_2_BYTES;
  frame.data.bytes[1] = index & 0xFF;
  frame.data.bytes[2] = index >> 8;
  frame.data.bytes[3] = subindex;
  frame.data.bytes[4] = data & 0xFF;
  frame.data.bytes[5] = data >> 8;
  frame.data.bytes[6] = 0;
  frame.data.bytes[7] = 0;
  bool sent = CAN_INTERFACE.sendFrame(frame);
  if (sent)
  {
    char buf[64];
    snprintf(buf, sizeof(buf), "[TX] SDO 2B %04X:%02X = %04X", index, subindex, data);
    Serial.println(buf);
  }
  return sent;
}

bool sendSDO_4Bytes(uint16_t index, uint8_t subindex, uint32_t data)
{
  CAN_FRAME frame;
  frame.id = SDO_REQUEST(SERVO_NODE_ID);
  frame.length = 8;
  frame.extended = false;
  frame.data.bytes[0] = WRITE_4_BYTES;
  frame.data.bytes[1] = index & 0xFF;
  frame.data.bytes[2] = index >> 8;
  frame.data.bytes[3] = subindex;
  frame.data.bytes[4] = data & 0xFF;
  frame.data.bytes[5] = (data >> 8) & 0xFF;
  frame.data.bytes[6] = (data >> 16) & 0xFF;
  frame.data.bytes[7] = (data >> 24) & 0xFF;
  bool sent = CAN_INTERFACE.sendFrame(frame);
  if (sent)
  {
    char buf[64];
    snprintf(buf, sizeof(buf), "[TX] SDO 4B %04X:%02X = %08lX", index, subindex, data);
    Serial.println(buf);
  }
  return sent;
}

void checkCANMessages()
{
  CAN_FRAME frame;
  if (CAN_INTERFACE.available())
  {
    CAN_INTERFACE.read(frame);

    if (frame.id == SDO_RESPONSE(SERVO_NODE_ID))
    {
      Serial.print("[RX] SDO Response: ");
      for (int i = 0; i < frame.length; i++)
      {
        char hexbuf[8];
        snprintf(hexbuf, sizeof(hexbuf), "%02X", frame.data.bytes[i]);
        Serial.print(hexbuf);
        Serial.print(" ");
      }
      Serial.println();
      waitingForResponse = false;
    }
    else if (frame.id == CHECK_RESPONSE(SERVO_NODE_ID))
    {
      Serial.println("[RX] Heartbeat detected!");
      init_can = true;
    }
    else if (frame.id == GUIDE_SENSOR_REQUEST(GUIDE_SENSOR_NODE_ID) && frame.data.bytes[0] == 0xCD && frame.data.bytes[1] == 0xAB)
    {
      Serial.println("[RX] Guide Sensor detected!");
      for (int i = 0; i < frame.length; i++)
      {
        char hexbuf[8];
        snprintf(hexbuf, sizeof(hexbuf), "%02X", frame.data.bytes[i]);
        Serial.print(hexbuf);
        Serial.print(" ");
      }
      Serial.println();
      waitingForResponse = false;
    }
    else
    {
      char idbuf[32];
      snprintf(idbuf, sizeof(idbuf), "[RX] ID 0x%03lX: ", frame.id);
      Serial.print(idbuf);
      for (int i = 0; i < frame.length; i++)
      {
        char hexbuf[8];
        snprintf(hexbuf, sizeof(hexbuf), "%02X", frame.data.bytes[i]);
        Serial.print(hexbuf);
        Serial.print(" ");
      }
      waitingForResponse = false;

      Serial.println();
    }
  }
}

// ====== TIMEOUT FUNCTION ======
bool isTimeout(unsigned long startTime, unsigned long timeoutMs)
{
  return (millis() - startTime) >= timeoutMs;
}

// ====== SEND SDO READ ======
bool sendSDO_Read(uint16_t index, uint8_t subindex)
{
  CAN_FRAME frame;
  frame.id = SDO_REQUEST(SERVO_NODE_ID);
  frame.length = 8;
  frame.extended = false;
  frame.data.bytes[0] = READ_REQUEST;
  frame.data.bytes[1] = index & 0xFF;
  frame.data.bytes[2] = index >> 8;
  frame.data.bytes[3] = subindex;
  frame.data.bytes[4] = 0x00;
  frame.data.bytes[5] = 0x00;
  frame.data.bytes[6] = 0x00;
  frame.data.bytes[7] = 0x00;
  bool sent = CAN_INTERFACE.sendFrame(frame);
  if (sent)
  {
    char buf[64];
    snprintf(buf, sizeof(buf), "[TX] Read SDO %04X:%02X", index, subindex);
    Serial.println(buf);
  }
  return sent;
}

// ====== SET SPEED ======
bool setSpeed(int16_t leftSpeed, int16_t rightSpeed)
{
  if (currentState != STATE_READY)
  {
    Serial.println("Driver not ready!");
    return false;
  }

  // Combine speeds (Little Endian format)
  uint32_t combinedSpeed = ((uint32_t)(rightSpeed & 0xFFFF) << 16) | (leftSpeed & 0xFFFF);
  char speedBuf[64];
  snprintf(speedBuf, sizeof(speedBuf), "Setting speeds: Left=%d RPM, Right=%d RPM", leftSpeed, rightSpeed);
  Serial.println(speedBuf);
  return sendSDO_4Bytes(OD_TARGET_VELOCITY, 0x03, combinedSpeed);
}

// ====== STATE MACHINE ======
void updateStateMachine()
{
  switch (currentState)
  {
  case INIT_SET_MODE:
    if (sendSDO_1Byte(OD_OPERATION_MODE, 0x00, MODE_PROFILE_VELOCITY))
    {
      waitingForResponse = true;
      stateStartTime = millis();
      currentState = INIT_ENABLE1;
    }
    break;
  case INIT_ENABLE1:
    if (!waitingForResponse || isTimeout(stateStartTime, 1000))
    {
      if (sendSDO_2Bytes(OD_ENABLE_MODE, 0x00, CTRL_SHUTDOWN))
      {
        waitingForResponse = true;
        stateStartTime = millis();
        currentState = INIT_ENABLE2;
      }
    }
    break;
  case INIT_ENABLE2:
    if (!waitingForResponse || isTimeout(stateStartTime, 1000))
    {
      if (sendSDO_2Bytes(OD_ENABLE_MODE, 0x00, CTRL_SWITCH_ON))
      {
        waitingForResponse = true;
        stateStartTime = millis();
        currentState = INIT_ENABLE3;
      }
    }
    break;
  case INIT_ENABLE3:
    if (!waitingForResponse || isTimeout(stateStartTime, 1000))
    {
      if (sendSDO_2Bytes(OD_ENABLE_MODE, 0x00, CTRL_ENABLE_OPERATION))
      {
        waitingForResponse = true;
        stateStartTime = millis();
        currentState = INIT_SENSOR_GUILD;
      }
    }
    break;
  case STATE_READY:
    Serial.println("🎉 Driver Ready! Sending demo velocity...");
    // setSpeed(-100, 100);
    // waitingForResponse = true;
    // stateStartTime = millis();
    currentState = INIT_START;
    break;
  case INIT_START:
    break;
  case STATE_ERROR:
    Serial.println("⚠️ Error in state machine");
  case INIT_SENSOR_GUILD:
    if (!waitingForResponse || isTimeout(stateStartTime, 1000))
    {
      if (sendGuideSensor_1Byte(CMD_GUIDE_SENSOR, READ_GUIDE_SENSOR, DATA_GUIDE_SENSOR))
      {
        waitingForResponse = true;
        stateStartTime = millis();
        currentState = STATE_READY;
      }
    }
    break;
  }
}

// ====== DECLARATIONS ======
void processCheckConnectWithTimeout(unsigned long timeout_ms)
{
  unsigned long start = millis();
  while (millis() - start < timeout_ms && !init_can)
  {
    checkCANMessages();
  }
  if (!init_can)
    Serial.println("❌ No heartbeat detected");
  else
    currentState = INIT_SET_MODE;
}
