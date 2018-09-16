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

#if !defined(_CANQUITTO_H_) && ( defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) )
#define _CANQUITTO_H_
#include <kinetis_flexcan.h>
#include <kinetis.h>
#include "Arduino.h"
#include "circular_buffer.h"
#include "IFCT.h"

#define CANQUITTO_BUFFER_SIZE 32

struct AsyncCQ {
  uint8_t node = 0;
  uint8_t packetid = 0;
};

typedef void (*_CQ_ptr)(const uint8_t* buffer, uint16_t length, AsyncCQ info);

class CANquitto {

  public:
    CANquitto() {;}
    bool begin(uint8_t node, uint32_t net = 0x8FFFFFF & 0x1FFE0000);
    uint8_t write(const uint8_t *array, uint32_t length, uint8_t _nodeID, uint8_t packetid = 0, uint32_t delay_send = 1000, uint32_t timeout = 2000);
    volatile uint32_t write_ack_valid = 1;
    volatile uint32_t write_id_validate = 0;
    uint32_t nodeNetID = 0x8FFFFFF & 0x1FFE0000;
    uint8_t nodeID = 1;
    volatile uint32_t is_processing = 0;
    static Circular_Buffer<uint8_t, CANQUITTO_BUFFER_SIZE, 12> primaryBuffer;
    static Circular_Buffer<uint8_t, CANQUITTO_BUFFER_SIZE*64, 12> secondaryBuffer;
    void onReceive(_CQ_ptr handler) { CANquitto::_handler = handler; }
    static _CQ_ptr _handler;

  private:
    bool _enabled = 1;

};

extern CANquitto Node;

#endif
