#include <esp_system.h>

#include <ArduinoJson.h>
StaticJsonDocument<256> jsonCmdReceive;
StaticJsonDocument<256> jsonInfoSend;

// device settings.
#define WHEEL_RX 18
#define WHEEL_TX 19

#define SERIAL_BAUDRATE 115200
#define WHEEL_BAUDRATE 115200

#define TIME_BETWEEN_CMD 4

const size_t packet_length = 10;     
uint8_t packet_move[packet_length] = {0x01, 0x64, 0xff, 0xce, 0x00, 0x00, 0x00, 0x00, 0x00, 0xda};

// -1: off
// 2000: wheel stops when there is no new cmd received in the past 2000ms.
int heartbeat_time_ms = -1;
unsigned long prev_time = millis();
bool stop_flag = false;

bool get_info_flag = false;


// func to print the packet_move data as HEX.
void print_packet(uint8_t *packet, size_t length) {
  for (size_t i = 0; i < length; i++) {
    if (i > 0) Serial.print(", ");
    Serial.print("0x");
    if (packet[i] < 0x10) Serial.print("0");
    Serial.print(packet[i], HEX);
  }
  Serial.println();
}


// CRC-8/MAXIM
uint8_t crc8_update(uint8_t crc, uint8_t data) {
  uint8_t i;
  crc = crc ^ data;
  for (i = 0; i < 8; ++i) {
    if (crc & 0x01) {
      crc = (crc >> 1) ^ 0x8c;
    } else {
      crc >>= 1;
    }
  }
  return crc;
}


// clear wheel serial buffer
void clear_wheel_buffer() {
  while (Serial1.available() > 0) {
    Serial1.read();
  }
}


// current loop, cmd: -32767 ~ 32767 -> -8 ~ 8 A (max current < 2.7A)
// speed loop, cmd: -200 ~ 200 rpm
// position loop, cmd: 0 ~ 32767 -> 0 ~ 360°

//    wherever the mode is set to position mode
//    the currently position is the 0 position and it moves to the goal position
//    at the direction as the shortest path.
void wheel_ctrl(uint8_t id, int cmd, uint8_t act) {
  packet_move[0] = id;
  packet_move[1] = 0x64;

  packet_move[2] = (cmd >> 8) & 0xFF;
  packet_move[3] = cmd & 0xFF;

  packet_move[4] = 0x00;
  packet_move[5] = 0x00;

  packet_move[6] = act;
  packet_move[7] = 0x00;

  // CRC-8/MAXIM
  uint8_t crc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    crc = crc8_update(crc, packet_move[i]);
  }
  packet_move[9] = crc;

  Serial1.write(packet_move, packet_length);
}


// change mode
// 1 - current loop
// 2 - speed loop
// 3 - position loop

void wheel_change_mode(uint8_t id, uint8_t mode) {
  packet_move[0] = id;
  packet_move[1] = 0xA0;
  packet_move[2] = 0x00;
  packet_move[3] = 0x00;
  packet_move[4] = 0x00;
  packet_move[5] = 0x00;
  packet_move[6] = 0x00;
  packet_move[7] = 0x00;
  packet_move[8] = 0x00;
  packet_move[9] = mode;
  Serial1.write(packet_move, packet_length);
  print_packet(packet_move, packet_length);
}


// get info
// feedback:
// 0  1    2        3        4       5       6    7  8     9
// ID MODE TORQUE_H TORQUE_L SPEED_H SPEED_L TEMP U8 ERROR CRC8
void wheel_get_info(uint8_t id) {
  packet_move[0] = id;

  get_info_flag = true;

  packet_move[1] = 0x74;

  packet_move[2] = 0x00;
  packet_move[3] = 0x00;

  packet_move[4] = 0x00;
  packet_move[5] = 0x00;
  packet_move[6] = 0x00;
  packet_move[7] = 0x00;

  packet_move[8] = 0x00;
  // CRC-8/MAXIM
  uint8_t crc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    crc = crc8_update(crc, packet_move[i]);
  }
  packet_move[9] = crc;
  Serial1.write(packet_move, packet_length);
}


// stop a single wheel.
void wheel_stop(uint8_t id) {
  wheel_ctrl(id, 0, 0);
}


// set the heartbeat time.
void set_heartbeat_time(int time_ms) {
  heartbeat_time_ms = time_ms;
}


// heartbeat ctrl
void heartbeat_ctrl() {
  if (heartbeat_time_ms == -1) {
    return;
  }
  unsigned long curr_time = millis();
  if (curr_time - prev_time > heartbeat_time_ms && !stop_flag) {
    wheel_stop(1);
    delay(TIME_BETWEEN_CMD);
    wheel_stop(2);
    delay(TIME_BETWEEN_CMD);
    wheel_stop(3);
    delay(TIME_BETWEEN_CMD);
    wheel_stop(4);
    delay(TIME_BETWEEN_CMD);
    stop_flag = true;
    Serial.println("Heartbeat Stop");
  }
}


// json cmds.
#include "json_cmd.h"

// uart ctrl funcs.
#include "uart_ctrl.h"


void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  Serial1.begin(WHEEL_BAUDRATE, SERIAL_8N1, WHEEL_RX, WHEEL_TX);

  // clear wheel buffer.
  clear_wheel_buffer();
}


void loop() {
  // heartbeat function.
  heartbeat_ctrl();

  // recving data from wheel.
  wheel_fb();

  // recving the json cmd from uart.
  serialCtrl();
}