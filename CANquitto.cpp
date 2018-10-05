/*
  MIT License

  Copyright (c) 2018 Antonio Alexander Brewer (tonton81) - https://github.com/tonton81

  Designed and tested for PJRC Teensy (3.2, 3.5, 3.6).

  Forum link : https://forum.pjrc.com/threads/53776-CANquitto?p=187492#post187492

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/


#include <CANquitto.h>
#include <kinetis_flexcan.h>
#include <kinetis.h>
#include "Arduino.h"
#include "IFCT.h"

CANquitto Node = CANquitto();
Circular_Buffer<uint32_t, CANQUITTO_BUFFER_SIZE, 4> CANquitto::payload_queue;
Circular_Buffer<uint32_t, 8, 3> CANquitto::nodeBus;
_CQ_ptr CANquitto::_handler = nullptr;
uint32_t CANquitto::nodeNetID = 0x8FFFFFF & 0x1FFE0000;
uint8_t CANquitto::nodeID = 1;
volatile uint32_t CANquitto::write_ack_valid = 1;
volatile uint32_t CANquitto::write_id_validate = 0;
bool CANquitto::enabled = 0;


bool CANquitto::begin(uint8_t node, uint32_t net) {
  if ( node && node < 128 && net & 0x1FFE0000 ) {
    nodeID = node;
    nodeNetID = net & 0x1FFE0000;
    return (enabled = 1);
  }
  return (enabled = 0);
}

uint8_t CANquitto::write(const uint8_t *array, uint32_t length, uint8_t dest_node, uint8_t packetid, uint32_t delay_send, uint32_t wait_time, IFCT& bus) {
  if ( !length ) return 0;

  static uint32_t node_bus[3] = { 0 };
  node_bus[0] = dest_node;
  if ( !CANquitto::nodeBus.find(node_bus, 3, 0, 0, 0) ) return 0;

  write_ack_valid = 0;
  write_id_validate = dest_node;

  length += 5;
  uint8_t buffer[length];
  memmove(&buffer[5], &array[0], length-5);
  buffer[0] = (length-5) >> 8;
  buffer[1] = (length-5);

  uint16_t checksum = 0;
  for ( uint32_t i = 5; i < length; i++ ) checksum ^= buffer[i];
  buffer[2] = checksum >> 8;
  buffer[3] = checksum;
  buffer[4] = packetid;

  uint16_t buf_levels = (int)ceil((float)length / 6);
  uint8_t buf[buf_levels][8];

  for ( uint16_t i = 0; i < buf_levels; i++ ) {
    if ( i < buf_levels - 1 ) {
      memmove(&buf[i][2], &buffer[i * 6], 6);
      buf[i][0] = (6 << 4) | (i >> 8);
      buf[i][1] = i;
      continue;
    }
    for ( uint8_t k = (length - (i * 6)) + 2; k < 8; k++ ) buf[i][k] = 0xAA;
    memmove(&buf[i][2], &buffer[i * 6], (length - (i * 6)));
    buf[i][0] = ((length - (i * 6)) << 4) | (i >> 8);
    buf[i][1] = i;
  }

  static CAN_message_t _send;
  _send.ext = _send.seq = 1;

  for ( uint16_t j = 0; j < buf_levels; j++ ) {
    if ( !j ) _send.id = nodeNetID | dest_node << 7 | nodeID | 1 << 14;
    else if ( j == ( buf_levels - 1 ) ) _send.id = nodeNetID | dest_node << 7 | nodeID | 3 << 14;
    else if ( j ) _send.id = nodeNetID | dest_node << 7 | nodeID | 2 << 14;
    memmove(&_send.buf[0], &buf[j][0], 8);
    bus.write(_send);
    delayMicroseconds(delay_send);
  }

  bool retry_once = 1;
  uint32_t timeout = millis();
  while ( write_ack_valid != 0x06 ) {
    if ( millis() - timeout > wait_time ) break;
    if ( write_ack_valid == 0x15 ) {
      if ( retry_once ) {
        retry_once = 0;
        timeout = millis();
        continue;
      }
      break;
    }
  }

  return (( write_ack_valid ) ? write_ack_valid : 0xFF);
}










bool CANquitto::isOnline(uint8_t node) {
  static uint32_t node_bus[3] = { 0 };
  node_bus[0] = node;
  return CANquitto::nodeBus.find(node_bus, 3, 0, 0, 0);
}





































void ext_output(const CAN_message_t &msg) {

  if ( ( msg.id & 0x1FFE0000 ) != ( Node.nodeNetID & 0x1FFE0000 ) ) return; /* reject unknown net frames */

  if ( msg.id == ((CANquitto::nodeNetID & 0x1FFE0000) | CANquitto::write_id_validate | CANquitto::nodeID << 7 | 4 << 14) ) {
    switch ( msg.buf[0] ) {
      case 0: { /* NODE TRANSFER RESPONSE ACK */
          CANquitto::write_ack_valid = msg.buf[1];
          break;
        }
    }
    return;
  }

  if ( ( (msg.id & 0x3F80) >> 7 ) == Node.nodeID ) { /* something is for this node! */
    static uint32_t buffer[4];
    buffer[0] = msg.id;
    buffer[1] = (uint32_t)(msg.buf[0] << 24) | msg.buf[1] << 16 | msg.buf[2] << 8 | msg.buf[3];
    buffer[2] = (uint32_t)(msg.buf[4] << 24) | msg.buf[5] << 16 | msg.buf[6] << 8 | msg.buf[7];
    buffer[3] = 0;
    if ( ((msg.id & 0x1C000) >> 14) == 1 ) buffer[3] = 0x1000000 | ((uint16_t)(msg.buf[0] & 0xF) | msg.buf[1]); // start frame of payload
    if ( ((msg.id & 0x1C000) >> 14) == 2 ) buffer[3] = 0x2000000 | ((uint16_t)(msg.buf[0] & 0xF) | msg.buf[1]); // continuation frame of payload
    if ( ((msg.id & 0x1C000) >> 14) == 3 ) buffer[3] = 0x3000000; // last frame of payload
    CANquitto::payload_queue.push_back(buffer,4);
  }
  else if ( ((msg.id & 0x3F80) >> 7) == 0 ) { /* global msgs */
    static uint32_t node[3] = { 0 };
    node[0] = msg.id & 0x7F;
    node[1] = msg.bus;
    node[2] = millis();
    if (!(CANquitto::nodeBus.replace(node, 3, 0, 0, 0)) ) CANquitto::nodeBus.push_back(node, 3);
  }

}













uint16_t ext_events() {
  static uint32_t buffer[4];
  static uint16_t find_payload_len;
  static uint16_t find_frame_count_max;
  static uint16_t find_crc;
  static uint8_t  packetID;


  static uint32_t notify = millis();
  if ( millis() - notify >= NODE_KEEPALIVE ) {
    notify = millis();
    static CAN_message_t notifier;
    notifier.ext = 1;
    notifier.id = ( CANquitto::nodeNetID | CANquitto::nodeID );
    Can0.write(notifier);
#if defined(__MK66FX1M0__)
    Can1.write(notifier);
#endif
  }


  static uint32_t node_scan[3];
  for ( uint16_t i = 0; i < CANquitto::nodeBus.size(); i++ ) {
    CANquitto::nodeBus.peek_front(node_scan, 3, i);
    if ( millis() - node_scan[2] > NODE_UPTIME_LIMIT ) CANquitto::nodeBus.findRemove(node_scan, 3, 0, 1, 2);
  }


  static CAN_message_t response;
  response.ext = 1;
  response.id = (CANquitto::nodeNetID | ((buffer[0] & 0x7F) << 7) | CANquitto::nodeID | (4 << 14));
  response.buf[1] = 0x15;

  buffer[3] = 0x3000000;
  if ( CANquitto::payload_queue.find(buffer, 4, 3, 3, 3) ) find_frame_count_max = (buffer[1] >> 16) & 0xFFF;
  else return 0;

  static uint32_t node_bus[3] = { 0 };
  node_bus[0] = buffer[0] & 0x7F;
  node_bus[1] = 0;
  CANquitto::nodeBus.find(node_bus, 3, 0, 0, 0);


  static IFCT& bus = Can0;
  if ( !node_bus[1] ) bus = Can0;
#if defined(__MK66FX1M0__)
  else bus = Can1;
#endif


  buffer[3] = 0x1000000;
  if ( CANquitto::payload_queue.find(buffer, 4, 3, 3, 3) ) {
    packetID = (buffer[2] >> 8) & 0xFF;
    find_payload_len = buffer[1] & 0xFFFF;
    find_crc = buffer[2] >> 16;
  }
  else {
    CANquitto::flush_node_frames(buffer[0]);
    bus.write(response);
    return 0;
  }


  uint16_t payload_pos = 1;
  uint16_t calc_crc = 0;
  uint8_t payload[find_payload_len];


  for ( uint16_t i = 0; i < find_frame_count_max; i++ ) {
    if ( i == 0 ) {
      buffer[3] = 0x1000000;
      if (CANquitto::payload_queue.find(buffer, 4, 3, 3, 3)) {
        payload[0] = buffer[2] & 0xFF;
        continue;
      }
      else {
        CANquitto::flush_node_frames(buffer[0]);
        bus.write(response);
        return 0;
      }
    }

    buffer[3] = 0x2000000 | i;
    if (CANquitto::payload_queue.find(buffer, 4, 3, 3, 3)) {
      payload[payload_pos + 0] = buffer[1] >> 8;
      payload[payload_pos + 1] = buffer[1] >> 0;
      payload[payload_pos + 2] = buffer[2] >> 24;
      payload[payload_pos + 3] = buffer[2] >> 16;
      payload[payload_pos + 4] = buffer[2] >> 8;
      payload[payload_pos + 5] = buffer[2] >> 0;
      payload_pos += 6;
    }
    else {
      CANquitto::flush_node_frames(buffer[0]);
      bus.write(response);
      return 0;
    }
  }
  buffer[3] = 0x3000000;
  if ( CANquitto::payload_queue.find(buffer, 4, 3, 3, 3) ) {
      if ( (buffer[1] >> 28) >= 0 ) payload[payload_pos + 0] = buffer[1] >> 8;
      if ( (buffer[1] >> 28) >= 1 ) payload[payload_pos + 1] = buffer[1] >> 0;
      if ( (buffer[1] >> 28) >= 2 ) payload[payload_pos + 2] = buffer[2] >> 24;
      if ( (buffer[1] >> 28) >= 3 ) payload[payload_pos + 3] = buffer[2] >> 16;
      if ( (buffer[1] >> 28) >= 4 ) payload[payload_pos + 4] = buffer[2] >> 8;
      if ( (buffer[1] >> 28) >= 5 ) payload[payload_pos + 5] = buffer[2] >> 0;
  }
  else {
    CANquitto::flush_node_frames(buffer[0]);
    bus.write(response);
    return 0;
  }


  for ( uint16_t i = 0; i < find_payload_len; i++ ) calc_crc ^= payload[i];

  CANquitto::flush_node_frames(buffer[0]);

  if ( calc_crc == find_crc ) {
    static AsyncCQ info;
    info.node = buffer[0] & 0x7F;
    info.packetid = packetID;
    info.bus = node_bus[1];
    if ( CANquitto::_handler ) CANquitto::_handler(payload, find_payload_len, info);
    response.buf[1] = 0x06;
  }
  bus.write(response);
  return 0;
}


void CANquitto::flush_node_frames(uint32_t canID) {
  static uint32_t buffer[4];
  buffer[0] = canID;
  buffer[0] = (buffer[0] & 0x1FFE3FFF) | 1 << 14;
  while ( CANquitto::payload_queue.findRemove(buffer, 12, 0, 0, 0) );
  buffer[0] = (buffer[0] & 0x1FFE3FFF) | 2 << 14;
  while ( CANquitto::payload_queue.findRemove(buffer, 12, 0, 0, 0) );
  buffer[0] = (buffer[0] & 0x1FFE3FFF) | 3 << 14;
  while ( CANquitto::payload_queue.findRemove(buffer, 12, 0, 0, 0) );
}
