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
    bool write(const uint8_t *array, uint32_t length, uint8_t _nodeID, uint8_t packetid = 0, uint32_t delay_send = 1000, uint32_t timeout = 2000);
    volatile uint32_t write_ack_valid = 1;
    volatile uint32_t write_id_validate = 0;
    uint32_t nodeNetID = 0x8FFFFFF & 0x1FFE0000;
    uint8_t nodeID = 1;
    static Circular_Buffer<uint8_t, CANQUITTO_BUFFER_SIZE, 12> primaryBuffer;
    static Circular_Buffer<uint8_t, CANQUITTO_BUFFER_SIZE*64, 12> secondaryBuffer;
    void onReceive(_CQ_ptr handler) { CANquitto::_handler = handler; }
    static _CQ_ptr _handler;

  private:
    bool _enabled = 1;

};

extern CANquitto Node;

#endif