/***************
 *
 * UPDIUart
 *
 * release site: https://github.com/askn37/UPDIUart/
 * maintainer: askn https://twitter.com/askn37
 */
#pragma once
#include <Arduino.h>

#define UPDI_RX_BUFFER_SIZE 32

// #define UPDI_DEBUG

#if defined(UPDI_DEBUG) && defined(ZINNIA_AVR)
#define PIN_LED_SEND PIN_LEDG
#define PIN_LED_RECV PIN_LEDB
#define LED_SEND(S) gpioOpenDrain(_send_led, S)
#define LED_RECV(S) gpioOpenDrain(_recv_led, S)
#else
#define LED_SEND(S)
#define LED_RECV(S)
#endif

#define UPDI_RECV_TIMEOUT 30
#define UPDI_BREAK_TIME 24600

/* UPDI protocol set b(765) */
#define UPDI_SYNCH   0x55
#define UPDI_ACK     0x40
#define UPDI_LDS     0x00
#define UPDI_STS     0x40
#define UPDI_LD      0x20
#define UPDI_ST      0x60
#define UPDI_LDCS    0x80
#define UPDI_STCS    0xC0
#define UPDI_REPEAT  0xA0

/* KEY : */
#define UPDI_KEY_64  0xE0
#define UPDI_KEY_128 0xE1

/* SIB : System Information Block */
#define UPDI_SIB_64  0xE4
#define UPDI_SIB_128 0xE5

/* UPDI addr size b(32) for LDS/STS */
#define UPDI_ADDR1  0x00
#define UPDI_ADDR2  0x04
#define UPDI_ADDR3  0x08

/* UPDI data size b(10) for LDS/STS/LD/ST */
#define UPDI_DATA1  0x00
#define UPDI_DATA2  0x01
#define UPDI_DATA3  0x02

/* UPDI ptr type b(32) for LD/ST */
#define UPDI_PTR_IMD 0x00
#define UPDI_PTR_INC 0x04
#define UPDI_PTR_REG 0x08

/* UPDI CS register b(3210) for LDCS/STCS */
#define UPDI_CS_STATUSA        0x00
#define UPDI_CS_STATUSB        0x01
#define UPDI_CS_CTRLA          0x02
#define UPDI_CS_CTRLB          0x03
#define UPDI_CS_ASI_KEY_STATUS 0x07
#define UPDI_CS_ASI_RESET_REQ  0x08
#define UPDI_CS_ASI_CTRLA      0x09
#define UPDI_CS_ASI_SYS_CTRLA  0x0A
#define UPDI_CS_ASI_SYS_STATUS 0x0B
#define UPDI_CS_ASI_CRC_STATUS 0x0C

/* _rx_status */
#define UPDI_STAT_RXCIF  0x80
#define UPDI_STAT_BUFOVF 0x40
#define UPDI_STAT_FERR   0x04
#define UPDI_STAT_PERR   0x02

/* NVMCTRL */
#define NVMCTRL_BASE 0x1000
#define NVM_CMD_ERWP 3
#define NVM_CMD_PBC  4
#define NVM_CMD_CHER 5
#define NVM_CMD_EEER 6
#define NVM_CMD_WFU  7

class UPDIUart_Class {

 protected:

  volatile USART_t * const _hwserial_module;
  volatile uint8_t const _txrx_pin;
  volatile uint8_t _rx_data;
  volatile uint8_t _rx_status;

  uint16_t _break_us;
  int16_t _rx_count = 0;
  uint8_t _rx_buffer[UPDI_RX_BUFFER_SIZE];
  bool _pullup;

#if defined(UPDI_DEBUG) && defined(ZINNIA_AVR)
  volatile uint8_t _send_led;
  volatile uint8_t _recv_led;
#endif

  size_t SetupLDS8 (uint16_t addr, size_t len);

 public:

  UPDIUart_Class (volatile USART_t *hwserial_module, uint8_t hwserial_txrx_pin, bool enable_pullup)
    : _hwserial_module(hwserial_module)
    , _txrx_pin(hwserial_txrx_pin)
    , _pullup(enable_pullup) {}

  void begin (long baud = 225000L);
  void end (void);
  inline uint8_t rxLast (void) { return _rx_data; }
  inline uint8_t rxError (void) { return _rx_status; }
  inline uint8_t* buffer (void) { return _rx_buffer; }

  void BeginTransaction (void);
  void EndTransaction (size_t len);

  void BREAK (uint16_t us = 0);
  inline void SEND (uint8_t data) { SEND(&data, 1); }
  void SEND (uint8_t* data, size_t len);
  int16_t RECV (void);

  int16_t START (void);
  void STOP (void);
  void RESET (void);

  int16_t LDCS (uint8_t cs);
  void STCS (uint8_t cs, uint8_t val);

  uint16_t LD8 (uint16_t addr);
  uint16_t LD16 (uint16_t addr);
  size_t LDS8 (uint16_t addr, size_t len);
  size_t LDS8 (uint16_t addr, uint8_t *data, size_t len);

  bool ST8 (uint16_t addr, uint8_t val);
  bool ST16 (uint16_t addr, uint16_t val);
  size_t STS8 (uint16_t addr, uint8_t *data, size_t len);

  size_t LDSIB (void);
  bool START_USERROW (void);
  bool FINAL_USERROW (void);
  bool ENABLE_NVMPROG (void);
  uint8_t NVM_WAIT (void);
  bool NVM_CMD (uint8_t cmd);
  bool ERASE_CHIP (void);
  bool ST_FUSE (uint16_t index, uint8_t data);
};

// end of header
