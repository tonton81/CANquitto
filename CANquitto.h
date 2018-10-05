#if !defined(_CANQUITTO_H_) && ( defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) )
#define _CANQUITTO_H_
#include <kinetis_flexcan.h>
#include <kinetis.h>
#include "Arduino.h"
#include "circular_buffer.h"
#include "IFCT.h"

#define CANQUITTO_BUFFER_SIZE 256
#define NODE_UPTIME_LIMIT 2500
#define NODE_KEEPALIVE 1000

struct AsyncCQ {
  uint8_t node = 0;
  uint8_t packetid = 0;
  bool bus = 0;
};

typedef void (*_CQ_ptr)(const uint8_t* buffer, uint16_t length, AsyncCQ info);

class CANquitto {

  public:
    constexpr CANquitto() {;}
    static bool begin(uint8_t node, uint32_t net = 0x8FFFFFF & 0x1FFE0000);
    static uint8_t write(const uint8_t *array, uint32_t length, uint8_t _nodeID, uint8_t packetid = 0, uint32_t delay_send = 1000, uint32_t timeout = 2000, IFCT& bus = Can0);
    static volatile uint32_t write_ack_valid;
    static volatile uint32_t write_id_validate;
    static uint32_t nodeNetID;
    static uint8_t nodeID;
    static Circular_Buffer<uint32_t, CANQUITTO_BUFFER_SIZE, 4> payload_queue;
    static Circular_Buffer<uint32_t, 8, 3> nodeBus;
    static void onReceive(_CQ_ptr handler) { CANquitto::_handler = handler; }
    static _CQ_ptr _handler;
    static bool enabled;
    static bool isOnline(uint8_t node);
    static void flush_node_frames(uint32_t canID);
  private:

};

extern CANquitto Node;

#endif
