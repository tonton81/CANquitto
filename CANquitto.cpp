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
Circular_Buffer<uint8_t, CANQUITTO_BUFFER_SIZE, 12> CANquitto::primaryBuffer;
Circular_Buffer<uint8_t, CANQUITTO_BUFFER_SIZE*64, 12> CANquitto::secondaryBuffer;
_CQ_ptr CANquitto::_handler = nullptr;



bool CANquitto::begin(uint8_t node, uint32_t net) {
  if ( node && node < 128 && net & 0x1FFE0000 ) {
    nodeID = node;
    nodeNetID = net & 0x1FFE0000;
    _enabled = 1;
    return 1;
  }
  _enabled = 0;
  return 0;
}

uint8_t CANquitto::write(const uint8_t *array, uint32_t length, uint8_t dest_node, uint8_t packetid, uint32_t delay_send, uint32_t wait_time) {
  if ( !length || length > 8100) return 0;

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

  CAN_message_t _send;
  _send.ext = _send.seq = 1;

  for ( uint16_t j = 0; j < buf_levels; j++ ) {
    if ( !j ) _send.id = nodeNetID | dest_node << 7 | nodeID | 1 << 14;
    else if ( j == ( buf_levels - 1 ) ) _send.id = nodeNetID | dest_node << 7 | nodeID | 3 << 14;
    else if ( j ) _send.id = nodeNetID | dest_node << 7 | nodeID | 2 << 14;
    memmove(&_send.buf[0], &buf[j][0], 8);
    Can0.write(_send);
    if ( j == (buf_levels - 2) ) write_ack_valid = 0;
    delayMicroseconds(delay_send);
  }

  uint32_t timeout = millis();
  while ( !write_ack_valid ) {
    if ( millis() - timeout > wait_time ) break;
  }

  if ( write_ack_valid ) return write_ack_valid;
  write_ack_valid = 0xFF; // keep it set
  return 0;
}















































void ext_output(const CAN_message_t &msg) {

  /* ######### REJECT UNKNOWN NODE FRAMES ######### */
  if ( ( msg.id & 0x1FFE0000 ) != ( Node.nodeNetID & 0x1FFE0000 ) ) return;


  /* ######### WE RECEIVED STATUS/RESPONSE UPDATES, UPDATE VARIABLES ######### */
  if ( msg.id == ((Node.nodeNetID & 0x1FFE0000) | Node.write_id_validate | Node.nodeID << 7 | 4 << 14) ) {
    switch ( msg.buf[0] ) {
      case 0: { /* NODE RESPONSE ACK/NAK */
          if ( Node.write_ack_valid ) break;
          Node.write_ack_valid = msg.buf[1];
          break;
        }
    }
    return;
  }


  /* ######### WE'RE RECEIVING A PAYLOAD FOR THIS NODE ######### */
  if ( ( (msg.id & 0x3F80) >> 7 ) == Node.nodeID ) { /* something is for this node! */
    uint8_t buf[12] = { (uint8_t)(msg.id >> 24), (uint8_t)(msg.id >> 16),
                        (uint8_t)(msg.id >> 8), (uint8_t)msg.id , msg.buf[0], msg.buf[1], msg.buf[2],
                        msg.buf[3], msg.buf[4], msg.buf[5], msg.buf[6], msg.buf[7]
                      };

    /* ######### IF WE GET A START FRAME, CLEAR CURRENT NODE FRAMES FROM BOTH QUEUES ######### */
    if ( ( ( ( msg.id & 0x1C000 ) >> 14) & 0x7 ) == 1 ) {
      uint32_t _available = CANquitto::primaryBuffer.size();
      uint32_t masked_id = msg.id & 0x1FFE3FFF;
      uint8_t transfer_buf[12];
      for ( uint32_t c = 0; c < _available; c++ ) {
        CANquitto::primaryBuffer.pop_front(transfer_buf, 12);
        if ( masked_id != ( msg.id & 0x1FFE3FFF ) ) CANquitto::primaryBuffer.push_back(transfer_buf, 12);
      }
      _available = CANquitto::secondaryBuffer.size();
      for ( uint32_t c = 0; c < _available; c++ ) {
        CANquitto::secondaryBuffer.pop_front(transfer_buf, 12);
        if ( masked_id != ( msg.id & 0x1FFE3FFF ) ) CANquitto::secondaryBuffer.push_back(transfer_buf, 12);
      }
    }

    /* ######### QUEUE THE FRAME ######### */
    if ( Node.is_processing ) CANquitto::primaryBuffer.push_back(buf, 12);
    else {
      CANquitto::secondaryBuffer.push_back(buf, 12);
      uint8_t transfer[12];
      uint32_t primary_available = CANquitto::primaryBuffer.size();
      for ( uint32_t p = 0; p < primary_available; p++ ) {
        CANquitto::primaryBuffer.pop_front(transfer, 12);
        CANquitto::secondaryBuffer.push_back(transfer, 12);
      }
    }

  }
}





uint16_t ext_events() {
  uint8_t search[12]; /* buffer is recycled throughout the entire function */

  /* ######### IF COMPLETED FRAME NOT FOUND, EXIT IMMEDIATELY ######### */
  search[2] = ( Node.nodeID >> 1) | ( 3 << 6 );
  if ( !(CANquitto::secondaryBuffer.find(search, 12, 2, 2, 2)) ) return 0;
  uint16_t limit = (uint16_t)( search[4] & 0xF ) | (search[5] + 1);

  /* ######### GOTO FIRST FRAME FOR VARIABLE SETTINGS, EXIT IMMEDIATELY IF ERROR ######### */
  search[2] = ( Node.nodeID >> 1) | ( 1 << 6 );
  if ( !(CANquitto::secondaryBuffer.find(search, 12, 2, 2, 2)) ) return 0;

  Node.is_processing = 1;

  uint32_t masked_id = (search[0] << 24 | search[1] << 16 | search[2] << 8 | search[3]) & 0x1FFE3FFF;
  uint32_t _id = 0;
  uint16_t find_len = ((uint16_t)search[6] << 8 | search[7]);
  uint16_t find_crc = ((uint16_t)search[8] << 8 | search[9]);

  uint16_t memmove_shift = 0;
  uint8_t find_packetid = search[10], payload_transfer[find_len];
  memmove(&payload_transfer[memmove_shift], &search[11], (search[4] >> 4) );
  memmove_shift += (search[4] >> 4) - 5;

  uint16_t crc_check = 0;
  CAN_message_t response;
  response.ext = response.seq = 1;
  response.id = Node.nodeNetID | (masked_id & 0x7F) << 7 | Node.nodeID | 4 << 14;

  for ( uint16_t i = 1; i < limit; i++ ) {
    search[4] = (search[4] & ~(0xF)) | ((i & 0xF00) >> 8);
    search[5] = i;

    if ( i < ( limit - 1 ) ) {
      if (CANquitto::secondaryBuffer.find(search, 12, 3, 4, 5)) {
        memmove(&payload_transfer[memmove_shift], &search[6], (search[4] >> 4) );
        memmove_shift += (search[4] >> 4);
      }
      else {
        response.buf[1] = 0x15;
        Can0.write(response);
        break; // ERROR! CLEAR NODE FRAMES!
      }
    }
    else {
      search[2] |= (3 << 6);
      search[5] = i;

      if (CANquitto::secondaryBuffer.find(search, 12, 2, 3, 5)) {
        memmove(&payload_transfer[memmove_shift], &search[6], (search[4] >> 4) );
        memmove_shift += (search[4] >> 4);

        for ( uint32_t i = 0; i < find_len; i++ ) crc_check ^= payload_transfer[i];

        if ( crc_check == find_crc ) {
          response.buf[1] = 0x06;
          Can0.write(response);
          AsyncCQ info;
          info.node = (masked_id & 0x7F);
          info.packetid = find_packetid;
          if ( CANquitto::_handler ) CANquitto::_handler(payload_transfer, find_len, info);
        }
        else {
          response.buf[1] = 0x15;
          Can0.write(response);
        }
        break;
      }
    }
  }

  /* ######### CLEAR NODE'S FRAMES FROM QUEUE ######### */
  uint32_t _available = CANquitto::secondaryBuffer.size();
  for ( uint32_t c = 0; c < _available; c++ ) {
    CANquitto::secondaryBuffer.pop_front(search, 12);
    _id = (search[0] << 24 | search[1] << 16 | search[2] << 8 | search[3]) & 0x1FFE3FFF;
    if ( masked_id != _id ) CANquitto::secondaryBuffer.push_back(search, 12);
  }

  Node.is_processing = 0;
  return 0;
}
