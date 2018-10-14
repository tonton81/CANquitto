#if !defined(_CANQUITTO_H_) && ( defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) )
#define _CANQUITTO_H_
#include <kinetis_flexcan.h>
#include <kinetis.h>
#include "Arduino.h"
#include "circular_buffer.h"
#include "IFCT.h"
#include "Stream.h"

#define MAX_PAYLOAD_SIZE 200
#define MAX_NODE_RECEIVING 2
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
    CANquitto(uint8_t nodeToControl);
    static bool begin(uint8_t node, uint32_t net = 0x8FFFFFF & 0x1FFE0000);
    static volatile int write_ack_valid;
    static volatile int write_id_validate;
    static volatile int serial_write_count[6];
    static volatile int serial_write_response;
    static volatile int digitalread_response;
    static volatile int analogread_response;
    static uint32_t nodeNetID;
    static uint32_t nodeID;
    static Circular_Buffer<uint8_t, MAX_NODE_RECEIVING * 16, 12> cq_isr_buffer;
    static Circular_Buffer<uint8_t, (uint32_t)pow(2, ceil(log(MAX_NODE_RECEIVING) / log(2))), MAX_PAYLOAD_SIZE> storage;
    static Circular_Buffer<uint32_t, 8, 3> nodeBus;
    static void onReceive(_CQ_ptr handler) { CANquitto::_handler = handler; }
    static _CQ_ptr _handler;
    static bool enabled;
    static bool isOnline(uint8_t node);
    static IFCT& node_bus(uint8_t node);
    static uint8_t sendMsg(const uint8_t *array, uint32_t length, uint8_t packetid = 0, uint32_t delay_send = 0, uint32_t timeout = 3000);
    static void digitalWriteFast(uint8_t pin, uint8_t state) { digitalWrite(pin, state); }
    static void digitalWrite(uint8_t pin, uint8_t state);
    static uint8_t digitalReadFast(uint8_t pin) { return digitalRead(pin); }
    static void pinMode(uint8_t pin, uint8_t mode);
    static int digitalRead(uint8_t pin);
    static void toggle(uint8_t pin);
    static int analogRead(uint8_t pin);
    static void analogReadResolution(unsigned int bits);

    class NodeFeatures {
      public:
        NodeFeatures(){;}
        size_t print(const char *p) { return write((const uint8_t*)p, strlen(p)); }
        size_t println(const char *p);
        size_t write(uint8_t val) { return write(&val, 1); }
        size_t write(const uint8_t *buf, size_t size);
        void analogRead(){;}
      private:
        uint8_t featuredNode;
        uint8_t serial_access;
        uint8_t spi_access;
        uint8_t wire_access;
        uint8_t port; // serial/wire/spi port
        friend CANquitto;
    };
    static NodeFeatures Serial;
    static NodeFeatures Serial1;
    static NodeFeatures Serial2;
    static NodeFeatures Serial3;
    static NodeFeatures Serial4;
    static NodeFeatures Serial5;
    static NodeFeatures Serial6;
    static NodeFeatures Wire;
    static NodeFeatures Wire1;
    static NodeFeatures Wire2;
    static NodeFeatures Wire3;
    static NodeFeatures SPI;
    static NodeFeatures SPI1;
    static NodeFeatures SPI2;

  private:
    CANquitto() {;}
    static CANquitto Node;

};

extern CANquitto Node;

#endif
