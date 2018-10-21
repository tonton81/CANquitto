#if !defined(_CANQUITTO_H_) && ( defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) )
#define _CANQUITTO_H_
#include "Arduino.h"
#include "IFCT.h"
#include "SPI.h"
#include "i2c_t3.h"
//#include "Wire.h"

#define MAX_PAYLOAD_SIZE 200
#define MAX_NODE_RECEIVING 2
#define NODE_UPTIME_LIMIT 3000
#define NODE_KEEPALIVE 500

enum node_status_messages {
  NODE_NOSTATUS,
  NODE_DETECT,
  NODE_LOST
};

struct AsyncCQ {
  uint8_t node = 0;
  uint8_t packetid = 0;
  bool bus = 0;
  node_status_messages state;
};

typedef void (*_CQ_ptr)(const uint8_t* buffer, uint16_t length, AsyncCQ info);
typedef void (*_CQ_detectptr)(AsyncCQ info);


#if defined(__MK20DX256__) && defined(I2C_T3_H)
#define ADD_MISSING_ENUMS_I2C_T3 1
typedef enum i2c_pins_define_missing {
  I2C_PINS_3_4 = 20,
  I2C_PINS_7_8,
  I2C_PINS_33_34,
  I2C_PINS_37_38,
  I2C_PINS_47_48,
  I2C_PINS_56_57
} i2c_pins_define_missing;
#endif

#if defined(__MK64FX512__) && defined(I2C_T3_H)
#define ADD_MISSING_ENUMS_I2C_T3 1
typedef enum i2c_pins_define_missing {
  I2C_PINS_29_30 = 20,
  I2C_PINS_26_31,
  I2C_PINS_56_57
} i2c_pins_define_missing;
#endif

#if defined(__MK66FX1M0__) && defined(I2C_T3_H)
#define ADD_MISSING_ENUMS_I2C_T3 1
typedef enum i2c_pins_define_missing {
  I2C_PINS_29_30 = 20,
  I2C_PINS_26_31,
} i2c_pins_define_missing;
#endif

#if !defined(I2C_T3_H)
typedef enum i2c_pins {
  I2C_PINS_3_4 = 0,
  I2C_PINS_7_8,
  I2C_PINS_16_17,
  I2C_PINS_18_19,
  I2C_PINS_29_30,
  I2C_PINS_26_31,
  I2C_PINS_33_34,
  I2C_PINS_37_38,
  I2C_PINS_47_48,
  I2C_PINS_56_57,
  I2C_PINS_DEFAULT,
  I2C_PINS_COUNT
} i2c_pins;
typedef enum i2c_stop {I2C_NOSTOP, I2C_STOP}i2c_stop;
typedef enum i2c_mode {I2C_MASTER = 0, I2C_SLAVE}i2c_mode;
typedef enum i2c_pullup {I2C_PULLUP_EXT = 0, I2C_PULLUP_INT}i2c_pullup;
typedef enum i2c_op_mode {I2C_OP_MODE_IMM = 0, I2C_OP_MODE_ISR, I2C_OP_MODE_DMA}i2c_op_mode;
typedef enum i2c_rate     {I2C_RATE_100  = 100000,
                   I2C_RATE_200  = 200000,
                   I2C_RATE_300  = 300000,
                   I2C_RATE_400  = 400000,
                   I2C_RATE_600  = 600000,
                   I2C_RATE_800  = 800000,
                   I2C_RATE_1000 = 1000000,
                   I2C_RATE_1200 = 1200000,
                   I2C_RATE_1500 = 1500000,
                   I2C_RATE_1800 = 1800000,
                   I2C_RATE_2000 = 2000000,
                   I2C_RATE_2400 = 2400000,
                   I2C_RATE_2800 = 2800000,
                   I2C_RATE_3000 = 3000000
} i2c_rate;

typedef enum i2c_pins_define_missing {
  unused_placeholder = 50
} i2c_pins_define_missing;


#endif








class CANquitto {

  public:
    CANquitto(uint8_t nodeToControl);
    static bool begin(uint8_t node, uint32_t net = 0x8FFFFFF & 0x1FFE0000);
    static volatile int write_ack_valid;
    static volatile uint32_t write_id_validate;
    static volatile int serial_write_count[7];
    static volatile int serial_write_response;
    static volatile int wire_write_count[4];
    static volatile int digitalread_response;
    static volatile int analogread_response;
    static volatile int available_response;
    static volatile int peek_response;
    static volatile int read_response;
    static volatile int readbuf_response_flag;
    static volatile int wire_response_flag;
    static volatile int spi_response_flag;
    static volatile int touchread_response_flag;
    static volatile uint8_t readbuf_response[6];
    static uint32_t nodeNetID;
    static uint32_t nodeID;
    static Circular_Buffer<uint8_t, MAX_NODE_RECEIVING * 16, 12> cq_isr_buffer;
    static Circular_Buffer<uint8_t, (uint32_t)pow(2, ceil(log(MAX_NODE_RECEIVING) / log(2))), MAX_PAYLOAD_SIZE> storage;
    static Circular_Buffer<uint32_t, 8, 3> nodeBus;
    static void onReceive(_CQ_ptr handler) { CANquitto::_handler = handler; }
    static void onDetect(_CQ_detectptr handler) { CANquitto::detect_handler = handler; }
    static _CQ_ptr _handler;
    static _CQ_detectptr detect_handler;
    static bool enabled;
    static bool isOnline(uint8_t node);
    static IFCT& node_bus(uint8_t node);
    uint8_t sendMsg(const uint8_t *array, uint32_t length, uint8_t packetid = 0, uint32_t delay_send = 0, uint32_t timeout = 3000);
    void digitalWriteFast(uint8_t pin, uint8_t state) { digitalWrite(pin, state); }
    void digitalWrite(uint8_t pin, uint8_t state);
    uint8_t digitalReadFast(uint8_t pin) { return digitalRead(pin); }
    void pinMode(uint8_t pin, uint8_t mode);
    int digitalRead(uint8_t pin);
    int touchRead(uint8_t pin);
    void toggle(uint8_t pin);
    int analogRead(uint8_t pin);
    void analogReadResolution(unsigned int bits);

    class NodeFeatures : public Stream {
      public:
        NodeFeatures(){;}
        virtual size_t print(const char *p) { return write((const uint8_t*)p, strlen(p)); }
        virtual size_t println(const char *p);
        virtual size_t write(uint8_t val) { return write(&val, 1); }
        virtual size_t write(const char *buf, size_t size) { return write((uint8_t*)buf,size); }
        virtual size_t write(const uint8_t *buf, size_t size);
        virtual int available();
        virtual int peek();
        virtual int read();
        virtual size_t read(uint8_t* buf, size_t size);
        virtual void flush() {;}
        void begin(uint8_t address);
        void begin();
        void begin(int baud);
        void beginTransmission(uint8_t address);
        void beginTransmission(int address);
        uint8_t endTransmission(void);
        uint8_t endTransmission(uint8_t sendStop);
        uint8_t endTransmission(i2c_stop sendStop, uint32_t timeout=0);
        void sendTransmission(i2c_stop sendStop=I2C_STOP);
        void sendRequest(uint8_t addr, size_t len, i2c_stop sendStop=I2C_STOP);
        uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop);
        uint8_t requestFrom(uint8_t address, uint8_t quantity);
        uint8_t requestFrom(int address, int quantity);
        uint8_t requestFrom(int address, int quantity, int sendStop);
        void setClock(uint32_t clock);
        int getClock(void);
        uint8_t getError(void);
        uint8_t done(void);
        uint8_t finish(uint32_t timeout=0);
        uint8_t getRxAddr(void);
        void setDefaultTimeout(uint32_t timeout);
        void resetBus(void);
        void setSDA(uint8_t pin);
        void setSCL(uint8_t pin);
        void setRX(uint8_t pin);
        void setTX(uint8_t pin, bool opendrain=false);
        void begin(i2c_mode mode, uint8_t address1, uint8_t pinSCL, uint8_t pinSDA, i2c_pullup pullup=I2C_PULLUP_EXT, uint32_t rate=400000, i2c_op_mode opMode=I2C_OP_MODE_ISR);
        void begin(i2c_mode mode, uint8_t address1, uint8_t address2, uint8_t pinSCL, uint8_t pinSDA, i2c_pullup pullup=I2C_PULLUP_EXT, uint32_t rate=400000, i2c_op_mode opMode=I2C_OP_MODE_ISR);
        void begin(i2c_mode mode, uint8_t address1, i2c_pins pins=I2C_PINS_DEFAULT, i2c_pullup pullup=I2C_PULLUP_EXT, uint32_t rate=400000, i2c_op_mode opMode=I2C_OP_MODE_ISR);
        void begin(i2c_mode mode, uint8_t address1, i2c_pins_define_missing pins, i2c_pullup pullup=I2C_PULLUP_EXT, uint32_t rate=400000, i2c_op_mode opMode=I2C_OP_MODE_ISR) {
          begin(mode, address1, (i2c_pins)pins, (i2c_pullup)pullup, rate, (i2c_op_mode)mode);
        }
        void begin(i2c_mode mode, uint8_t address1, uint8_t address2, i2c_pins pins=I2C_PINS_DEFAULT, i2c_pullup pullup=I2C_PULLUP_EXT, uint32_t rate=400000, i2c_op_mode opMode=I2C_OP_MODE_ISR);
        void begin(i2c_mode mode, uint8_t address1, uint8_t address2, i2c_pins_define_missing pins, i2c_pullup pullup=I2C_PULLUP_EXT, uint32_t rate=400000, i2c_op_mode opMode=I2C_OP_MODE_ISR) {
          begin(mode, address1, address2, (i2c_pins)pins, (i2c_pullup)pullup, rate, (i2c_op_mode)opMode);
        }
        uint8_t pinConfigure(i2c_pins pins, i2c_pullup pullup=I2C_PULLUP_EXT);
        uint8_t pinConfigure(i2c_pins_define_missing pins, i2c_pullup pullup=I2C_PULLUP_EXT) { return pinConfigure((i2c_pins)pins, (i2c_pullup)pullup); }
        uint8_t pinConfigure(uint8_t pinSCL, uint8_t pinSDA, i2c_pullup pullup=I2C_PULLUP_EXT);
        uint8_t setOpMode(i2c_op_mode opMode);
        void setRate(uint32_t busFreq = F_BUS, uint32_t i2cFreq = 400000) { setClock(i2cFreq); }
        uint8_t transfer(uint8_t data);
        uint16_t transfer16(uint16_t data);
        void endTransaction(void);
        void beginTransaction(uint32_t speed, uint8_t msb, uint8_t mode);
        void setMOSI(uint8_t pin);
        void setMISO(uint8_t pin);
        void setSCK(uint8_t pin);
        uint8_t setCS(uint8_t pin);

      private:
        uint8_t featuredNode;
        uint8_t serial_access;
        uint8_t spi_access;
        uint8_t wire_access;
        uint8_t port; // serial/wire/spi port
        friend CANquitto;
    };
    NodeFeatures Serial;
    NodeFeatures Serial1;
    NodeFeatures Serial2;
    NodeFeatures Serial3;
    NodeFeatures Serial4;
    NodeFeatures Serial5;
    NodeFeatures Serial6;
    NodeFeatures Wire;
    NodeFeatures Wire1;
    NodeFeatures Wire2;
    NodeFeatures Wire3;
    NodeFeatures SPI;
    NodeFeatures SPI1;
    NodeFeatures SPI2;

  private:
    CANquitto() {;}
    static CANquitto Node;

};

extern CANquitto Node;

#endif
