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

bool CANquitto::write(const uint8_t *array, uint32_t length, uint8_t dest_node, uint8_t packetid, uint32_t delay_send, uint32_t wait_time) {
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
    delayMicroseconds(delay_send);
    if ( j == (buf_levels - 2) ) write_ack_valid = 0;
  }

  uint32_t timeout = millis();
  while ( !write_ack_valid ) {
    if ( millis() - timeout > wait_time ) {
      break;
    }
  }

  if ( write_ack_valid == 0x06 ) return 1;
  write_ack_valid = 1; // keep it set
  return 0;
}















































void ext_output(const CAN_message_t &msg) {

  /* ######### REJECT UNKNOWN NODE FRAMES ######### */
  if ( ( msg.id & 0x1FFE0000 ) != ( Node.nodeNetID & 0x1FFE0000 ) ) return;


  /* ######### WE RECEIVED STATUS/RESPONSE UPDATES, UPDATE VARIABLES ######### */
  if ( msg.id == ((Node.nodeNetID & 0x1FFE0000) | Node.write_id_validate | Node.nodeID << 7 | 4 << 14) ) {

    if ( Node.write_ack_valid ) return;

    switch ( msg.buf[0] ) {
      case 0: { /* NODE TRANSFER RESPONSE ACK */
          Node.write_ack_valid = msg.buf[1];
          break;
        }

    }
    return;
  }

  if ( ( (msg.id & 0x3F80) >> 7 ) == Node.nodeID ) { /* something is for this node! */
    uint8_t buf[12] = { (uint8_t)(msg.id >> 24), (uint8_t)(msg.id >> 16),
                        (uint8_t)(msg.id >> 8), (uint8_t)msg.id , msg.buf[0], msg.buf[1], msg.buf[2],
                        msg.buf[3], msg.buf[4], msg.buf[5], msg.buf[6], msg.buf[7]
                      };


    /* ######### IF WE GET A START FRAME, CLEAR CURRENT NODE FRAMES FROM BOTH QUEUES ######### */
    if ( ( ( ( msg.id & 0x1C000 ) >> 14) &0x7 ) == 1 ) {
      uint32_t _available = CANquitto::primaryBuffer.size();
      uint32_t masked_id = msg.id & 0x1FFE3FFF;
      uint8_t transfer_buf[12];
      for ( uint32_t c = 0; c < _available; c++ ) {
        CANquitto::primaryBuffer.pop_front(transfer_buf, 12);
        if ( masked_id != msg.id & 0x1FFE3FFF ) CANquitto::primaryBuffer.push_back(transfer_buf, 12);
      }
      _available = CANquitto::secondaryBuffer.size();
      for ( uint32_t c = 0; c < _available; c++ ) {
        CANquitto::secondaryBuffer.pop_front(transfer_buf, 12);
        if ( masked_id != msg.id & 0x1FFE3FFF ) CANquitto::secondaryBuffer.push_back(transfer_buf, 12);
      }
    }



    CANquitto::primaryBuffer.push_back(buf, 12);
  }
}

uint16_t ext_events() {
  uint8_t transfer_buf[12]; /* buffer is recycled throughout the entire function */

  /* ################## TAKE ALL DATA FROM PRIMARY BUFFER (CONSUMER ONLY) ######################## */
  /* ################## AND PUSH IT TO SECONDARY BUFFER (CONSUMER & PRODUCER) #################### */
  uint32_t primary_available = CANquitto::primaryBuffer.size();
  for ( uint32_t p = 0; p < primary_available; p++ ) {
    CANquitto::primaryBuffer.pop_front(transfer_buf, 12);
    CANquitto::secondaryBuffer.push_back(transfer_buf, 12);
  }

  uint32_t cycle_size = 0, payload_checksum = 0, checksum = 0, payload_length = 0, _id = 0, masked_id = 0;
  uint32_t _available = CANquitto::secondaryBuffer.size();

  /* ####### WE START ROLLING THE BUFFER UNTIL THE LAST FRAME IS FOUND (END OF NODE TRANSFER) ####### */
  while ( cycle_size < _available ) {
    cycle_size++;
    CANquitto::secondaryBuffer.pop_front(transfer_buf, 12);
    CANquitto::secondaryBuffer.push_back(transfer_buf, 12);

    /* ################## WE FOUND A COMPLETED TRANSFER! ATTEMPTING REBUILD... #################### */
    if ( ((((uint16_t)transfer_buf[1] << 8 | transfer_buf[2]) >> 6) & 0x7) == 3 ) {
      /* ####### HERE WE MODIFY THE BITS NEEDED TO DO RUN A CIRCULAR_BUFFER FIND MATCHING ARRAY ####### */
      transfer_buf[2] &= ~(1 << 7);
      transfer_buf[5] = 0;

      /* ################## IF WE HAVE A MATCHING FIRST FRAME, BEGIN PROCESSING #################### */
      if (CANquitto::secondaryBuffer.find(transfer_buf, 12, 0, 1, 2, 3, 5)) {

        /* ######### THIS IS NEEDED SINCE THE FIND METHOD SEARCHES THE INDICE AS WHOLE ######### */
        /* ######### AND NOT BY BITS. WE NEED COUNTER TO MATCH 12 LSB BITS OF 2 BYTES. ######### */
        /* ######### NOT JUST THE SECOND BYTE (FOR LARGE ARRAYS BIGGER THAN 256 BYTES! ######### */
        if ( ((((uint16_t)transfer_buf[4]) & 0xF) << 8 | transfer_buf[5]) != 0 ) continue; /* 12 bits must be == 0 */

        /* ##### WE MASK OUT THE CONTROL BITS FROM THE ID AND LOCK ONLY TO THIS SPECIFIC NODE'S FRAMES ##### */
        masked_id = transfer_buf[0] << 24 | transfer_buf[1] << 16 | transfer_buf[2] << 8 | transfer_buf[3];
        masked_id &= 0x1FFE3FFF;

        /* ######### WE COPY THE LWN & CRC FROM THE FIRST FRAME SEQUENCE WE FOUND ABOVE ######### */
        payload_length = ((uint16_t)transfer_buf[6] << 8 | transfer_buf[7]);
        payload_checksum = ((uint16_t)transfer_buf[8] << 8 | transfer_buf[9]);
        uint8_t payload[payload_length];
        uint8_t packetid = transfer_buf[10];
        uint32_t shift_array = 0;

        _available = CANquitto::secondaryBuffer.size();
        for ( uint32_t i = 0; i < _available; i++ ) {

          /* ##### WE THEN LOOK FOR THE CONSECUTIVE FRAMES IN ORDER, LOOK FOR CORRECT 12 BIT FIELD ##### */
          for ( uint32_t o = 0; o < _available; o++ ) {
            _id = transfer_buf[0] << 24 | transfer_buf[1] << 16 | transfer_buf[2] << 8 | transfer_buf[3];
            _id &= 0x1FFE3FFF;
            if ( ((((uint16_t)transfer_buf[4]) & 0xF) << 8 | transfer_buf[5]) != i || ( masked_id != _id ) ) {
              CANquitto::secondaryBuffer.pop_front(transfer_buf, 12);
              CANquitto::secondaryBuffer.push_back(transfer_buf, 12);
            }
            else break;
          }


          /* ######### HERE WE MAKE SURE WE HAVE THE CORRECT SPECIFIC NODE FOR RECEPTION ######### */
          _id = transfer_buf[0] << 24 | transfer_buf[1] << 16 | transfer_buf[2] << 8 | transfer_buf[3];
          _id &= 0x1FFE3FFF;


          /* ##### IF THE 12 BITS MATCH SEQUENCE AND THE NODE MASK MATCHES THE COMPLETE MESSAGE FRAME ##### */
          if ( ((((uint16_t)transfer_buf[4]) & 0xF) << 8 | transfer_buf[5]) == i && ( masked_id == _id ) ) {
            /* ######### WE ATTEMPT TO REMOVE THE FRAME FROM QUEUE WHILE PROCESSING ######### */
            /* ######### THE FRAME COULD BE IN FRONT OR BACK OF QUEUE, IT DEPENDS IF THE QUEUE ROLLED ######### */
            if ( CANquitto::secondaryBuffer.peek_front()[5] == transfer_buf[5] ) CANquitto::secondaryBuffer.pop_front();
            if ( CANquitto::secondaryBuffer.peek_back()[5] == transfer_buf[5] ) CANquitto::secondaryBuffer.pop_back();

            /* ##### FOR THE FIRST FRAME, WE REMOVE THE APPENDED LEN AND CRC FROM THE TRANSFER PACKET PAYLOAD ##### */
            if ( !i ) {
              memmove(&payload[shift_array], &transfer_buf[11], (transfer_buf[4] >> 4) );
              shift_array += (transfer_buf[4] >> 4) - 5;
            }
            /* ##### FOR THE REST OF THE SEQUENCES, RUN THEM AS NORMAL ##### */
            else {
              memmove(&payload[shift_array], &transfer_buf[6], (transfer_buf[4] >> 4) );
              shift_array += (transfer_buf[4] >> 4);
            }
          }
        }

        /* ######### WE REMOVE ANY STRAY FRAMES THAT MAY BE IN BUFFER FOR THAT SPECIFIC NODE ######### */
        /* ######### STALE FRAMES COULD BE INCOMPLETE PAYLOADS FROM A NODE WHEN IT WAS RESET ######### */
        /* ######### THUS THE FRAMES SIT IN QUEUE WHILE NEW ONES ENTER ON NEXT TRANSMISSION. ######### */
        /* ######### NOTE: YOU MAY GET A NAK AS AFTER IT FAILS CRC USING STALE FRAMES ON BOOT ######### */
        /* #########       THE REST OF IT'S FRAMES WOULD BE DROPPED AND START FRESH AGAIN ######### */
        _available = CANquitto::secondaryBuffer.size();
        for ( uint32_t c = 0; c < _available; c++ ) {
          CANquitto::secondaryBuffer.pop_front(transfer_buf, 12);
          _id = transfer_buf[0] << 24 | transfer_buf[1] << 16 | transfer_buf[2] << 8 | transfer_buf[3];
          _id &= 0x1FFE3FFF;
          if ( masked_id != _id ) CANquitto::secondaryBuffer.push_back(transfer_buf, 12);
        }

        /* ######### WE BEGIN A MESSAGE FRAME TO SEND AN ACK/NAK RESPONSE TO NODE ######### */
        CAN_message_t msg;
        msg.ext = msg.seq = 1;
        msg.id = Node.nodeNetID | (masked_id & 0x7F) << 7 | Node.nodeID | 4 << 14;


        /* ######### IF THE CHECKSUM PASSES, SEND BACK AN ACK AND FIRE A CALLBACK ######### */
        for ( uint32_t i = 0; i < payload_length; i++ ) checksum ^= payload[i];
        if ( checksum == payload_checksum ) {
          msg.buf[1] = 0x06;
          Can0.write(msg);
          AsyncCQ info;
          info.node = (masked_id & 0x7F);
          info.packetid = packetid;
          if ( CANquitto::_handler ) CANquitto::_handler(payload, payload_length, info);
          return 1;
        }
        /* ######### IF THE CHECKSUM FAILS, SEND BACK A NAK ######### */
        msg.buf[1] = 0x15;
        Can0.write(msg);
        return 0;
      }
    }
  }
  return 0;
}
