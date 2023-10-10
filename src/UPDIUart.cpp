/***************
 *
 * UPDIUart
 *
 * release site: https://github.com/askn37/UPDIUart/
 * maintainer: askn https://twitter.com/askn37
 */
#include <Arduino.h>
#include <util/atomic.h>
#include "UPDIUart.h"

const static uint8_t erase_key[10]     = { UPDI_SYNCH, UPDI_KEY_64, 0x65, 0x73, 0x61, 0x72, 0x45, 0x4D, 0x56, 0x4E };
const static uint8_t nvmprog_key[10]   = { UPDI_SYNCH, UPDI_KEY_64, 0x20, 0x67, 0x6F, 0x72, 0x50, 0x4D, 0x56, 0x4E };
const static uint8_t urowwrite_key[10] = { UPDI_SYNCH, UPDI_KEY_64, 0x65, 0x74, 0x26, 0x73, 0x55, 0x4D, 0x56, 0x4E };

void UPDIUart_Class::begin (long baud) {
  if (baud > (F_CPU >> 4)) baud = F_CPU >> 4;
  int32_t baud_setting = (((F_CPU << 4) / baud) + 1) >> 1;  // RXMODE=CLK2X
  int8_t sigrow_val = (FUSE.OSCCFG & FUSE_FREQSEL_gm) == FUSE_FREQSEL1_bm ? SIGROW.OSC20ERR3V : SIGROW.OSC16ERR3V;
  baud_setting += (baud_setting * sigrow_val) >> 10;
  _break_us = 25000000 / baud;

#if defined(UPDI_DEBUG) && defined(ZINNIA_AVR)
  _send_led = gpioNumber(PIN_LED_SEND);
  _recv_led = gpioNumber(PIN_LED_RECV);
#endif

  if (_pullup) pinMode(_txrx_pin, INPUT_PULLUP);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    (*_hwserial_module).CTRLA = USART_LBME_bm;              /* Loop-back Mode Enable  */
    (*_hwserial_module).CTRLB = USART_ODME_bm               /* Open Drain Mode Enable */
                              | USART_TXEN_bm               /* Transmitter Enable     */
                              | USART_RXEN_bm               /* Receiver Enable        */
                              | USART_RXMODE_CLK2X_gc;      /* Receiver Mode : DOUBLE */
    (*_hwserial_module).BAUD  = (uint16_t) baud_setting;    /* Master baudrate          : baud         */
    (*_hwserial_module).CTRLC = USART_CMODE_ASYNCHRONOUS_gc /* USART Communication Mode : ASYNCHRONOUS */
                              | USART_PMODE_EVEN_gc         /* Parity Mode              : EVEN         */
                              | USART_SBMODE_2BIT_gc        /* Stop Bit Mode            : 2BIT         */
                              | USART_CHSIZE_8BIT_gc;       /* Character Size           : 8BIT         */
  }
}

void UPDIUart_Class::end (void) {
  RESET();
  STOP();
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    (*_hwserial_module).STATUS =
    (*_hwserial_module).CTRLC =
    (*_hwserial_module).CTRLB =
    (*_hwserial_module).CTRLA =
    (*_hwserial_module).BAUD = 0;
  }
  pinMode(_txrx_pin, _pullup ? INPUT_PULLUP : INPUT);
}

void UPDIUart_Class::BeginTransaction (void) {
  _rx_count = 0;
}

void UPDIUart_Class::EndTransaction (size_t len) {
  LED_RECV(LOW);
  uint32_t ms = millis();
  do {
    if ((*_hwserial_module).STATUS & USART_RXCIF_bm) {
      _rx_status |= (*_hwserial_module).RXDATAH ^ 0x80;
      _rx_data = (*_hwserial_module).RXDATAL;
      if (_rx_count < UPDI_RX_BUFFER_SIZE) {
        if (_rx_count >= 0) _rx_buffer[_rx_count] = _rx_data;
        _rx_count++;
      }
      if (_rx_count == len) break;
      ms = millis();
    }
  } while ((millis() - ms) <= UPDI_RECV_TIMEOUT || _rx_count < len);
  LED_RECV(HIGH);
}

void UPDIUart_Class::BREAK (uint16_t us) {
  /* SYNCH after  : 12 clock LOW */
  /* SYNCH before : 24.6ms LOW   */
  if (!us) us = _break_us;
  LED_SEND(LOW);

  /* Disable UART */
  uint8_t savea = (*_hwserial_module).CTRLA;
  uint8_t saveb = (*_hwserial_module).CTRLB;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    (*_hwserial_module).STATUS =
    (*_hwserial_module).CTRLA =
    (*_hwserial_module).CTRLB = 0;
  }

  /* Manual LOW sync logic */
  pinMode(_txrx_pin, OUTPUT);
  delayMicroseconds(us);

  /* Retore PULLUP */
  pinMode(_txrx_pin, _pullup ? INPUT_PULLUP : INPUT);

  /* Target PULLUP waiting */
  while (!digitalRead(_txrx_pin));

  /* Restore UART */
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    (*_hwserial_module).CTRLA = savea;
    (*_hwserial_module).CTRLB = saveb;
  }

  LED_SEND(HIGH);
}

void UPDIUart_Class::SEND (uint8_t* data, size_t len) {
  LED_SEND(LOW);
  do {
    while (!((*_hwserial_module).STATUS & USART_DREIF_bm));
    (*_hwserial_module).TXDATAL = *data++;
    _rx_count--;
    if ((*_hwserial_module).STATUS & USART_RXCIF_bm) {
      _rx_status |= (*_hwserial_module).RXDATAH ^ 0x80;
      _rx_data = (*_hwserial_module).RXDATAL;
      _rx_count++;
    }
  } while (--len);
  LED_SEND(HIGH);
}

int16_t UPDIUart_Class::RECV (void) {
  if (_rx_count <= 0) EndTransaction(1);
  if (_rx_count > 0 && !_rx_status) return _rx_data;
  return -1;
}

const static uint8_t ldcs_sta[2] = { UPDI_SYNCH, UPDI_LDCS|UPDI_CS_STATUSA };
int16_t UPDIUart_Class::START (void) {
  BREAK(UPDI_BREAK_TIME);
  BeginTransaction();
  SEND((uint8_t*)ldcs_sta, sizeof(ldcs_sta));
  return RECV();
}

const static uint8_t stcs_ctrlb[2] = { UPDI_SYNCH, UPDI_STCS|UPDI_CS_CTRLB };
void UPDIUart_Class::STOP (void) {
  BeginTransaction();
  SEND((uint8_t*)stcs_ctrlb, sizeof(stcs_ctrlb));
  SEND(4);
  EndTransaction(0);
}

const static uint8_t stcs_asi_rst[2] = { UPDI_SYNCH, UPDI_STCS|UPDI_CS_ASI_RESET_REQ };
void UPDIUart_Class::RESET (void) {
  BeginTransaction();
  SEND((uint8_t*)stcs_asi_rst, sizeof(stcs_asi_rst));
  SEND(0x59);
  SEND((uint8_t*)stcs_asi_rst, sizeof(stcs_asi_rst));
  SEND(0);
  EndTransaction(0);
  BREAK(UPDI_BREAK_TIME);
}

int16_t UPDIUart_Class::LDCS (uint8_t cs) {
  BeginTransaction();
  SEND(UPDI_SYNCH);
  SEND(UPDI_LDCS | cs);
  return RECV();
}

void UPDIUart_Class::STCS (uint8_t cs, uint8_t val) {
  BeginTransaction();
  SEND(UPDI_SYNCH);
  SEND(UPDI_STCS | cs);
  SEND(val);
  EndTransaction(0);
}

const static uint8_t lds_2_1[2]  = { UPDI_SYNCH, UPDI_LDS|UPDI_ADDR2|UPDI_DATA1 };
uint16_t UPDIUart_Class::LD8 (uint16_t addr) {
  BeginTransaction();
  SEND((uint8_t*)lds_2_1, sizeof(lds_2_1));
  SEND((uint8_t*)&addr, sizeof(addr));
  return RECV();
}

const static uint8_t lds_2_2[2]  = { UPDI_SYNCH, UPDI_LDS|UPDI_ADDR2|UPDI_DATA2 };
uint16_t UPDIUart_Class::LD16 (uint16_t addr) {
  BeginTransaction();
  SEND((uint8_t*)lds_2_2, sizeof(lds_2_2));
  SEND((uint8_t*)&addr, sizeof(addr));
  EndTransaction(2);
  return (_rx_buffer[0] | ((uint16_t)_rx_buffer[1] << 8));
}

const static uint8_t sts_2_1[2]  = { UPDI_SYNCH, UPDI_STS|UPDI_ADDR2|UPDI_DATA1 };
bool UPDIUart_Class::ST8 (uint16_t addr, uint8_t val) {
  BeginTransaction();
  SEND((uint8_t*)sts_2_1, sizeof(sts_2_1));
  SEND((uint8_t*)&addr, sizeof(addr));
  if (UPDI_ACK == RECV()) {
    BeginTransaction();
    SEND(val);
    if (UPDI_ACK == RECV()) return true;
  }
  return false;
}

const static uint8_t sts_2_2[2]  = { UPDI_SYNCH, UPDI_STS|UPDI_ADDR2|UPDI_DATA2 };
bool UPDIUart_Class::ST16 (uint16_t addr, uint16_t val) {
  BeginTransaction();
  SEND((uint8_t*)sts_2_2, sizeof(sts_2_2));
  SEND((uint8_t*)&addr, sizeof(addr));
  if (UPDI_ACK == RECV()) {
    BeginTransaction();
    SEND(val);
    SEND(val >> 8);
    if (UPDI_ACK == RECV()) return true;
  }
  return false;
}

const static uint8_t st_ptr_2[2] = { UPDI_SYNCH, UPDI_ST|UPDI_PTR_REG|UPDI_DATA2 };
const static uint8_t repeat_1[2] = { UPDI_SYNCH, UPDI_REPEAT|UPDI_DATA1 };
const static uint8_t ld_inc_1[2] = { UPDI_SYNCH, UPDI_LD|UPDI_PTR_INC|UPDI_DATA1 };
size_t UPDIUart_Class::SetupLDS8 (uint16_t addr, size_t len) {
  BeginTransaction();
  SEND((uint8_t*)st_ptr_2, sizeof(st_ptr_2));
  SEND((uint8_t*)&addr, sizeof(addr));
  if (!UPDI_ACK == RECV()) return 0;
  BeginTransaction();
  SEND((uint8_t*)repeat_1, sizeof(repeat_1));
  SEND(len - 1);
  SEND((uint8_t*)ld_inc_1, sizeof(ld_inc_1));
  return 1;
}

size_t UPDIUart_Class::LDS8 (uint16_t addr, size_t len) {
  if (!SetupLDS8(addr, len)) return 0;
  EndTransaction(len);
  return _rx_count;
}

size_t UPDIUart_Class::LDS8 (uint16_t addr, uint8_t *data, size_t len) {
  if (!SetupLDS8(addr, len)) return 0;
  size_t _count = 0;
  do {
    int16_t c = RECV();
    if (c < 0) break;
    *data++ = (uint8_t) c;
    BeginTransaction();
    _count++;
  } while (--len);
  return _count;
}

const static uint8_t st_inc_1[2] = { UPDI_SYNCH, UPDI_ST|UPDI_PTR_INC|UPDI_DATA1 };
size_t UPDIUart_Class::STS8 (uint16_t addr, uint8_t *data, size_t len) {
  BeginTransaction();
  SEND((uint8_t*)st_ptr_2, sizeof(st_ptr_2));
  SEND((uint8_t*)&addr, sizeof(addr));
  if (!UPDI_ACK == RECV()) return 0;
  BeginTransaction();
  SEND((uint8_t*)repeat_1, sizeof(repeat_1));
  SEND(len - 1);
  SEND((uint8_t*)st_inc_1, sizeof(st_inc_1));
  size_t _count = 0;
  do {
    SEND(*data++);
    if (!UPDI_ACK == RECV()) return _count;
    BeginTransaction();
    _count++;
  } while (--len);
  return _count;
}

const static uint8_t ldsib_2[2]  = { UPDI_SYNCH, UPDI_SIB_128 };
size_t UPDIUart_Class::LDSIB (void) {
  BeginTransaction();
  SEND((uint8_t*)ldsib_2, sizeof(ldsib_2));
  EndTransaction(16);
  return _rx_count;
}

const static uint8_t ldcs_key_stat[2]  = { UPDI_SYNCH, UPDI_LDCS|UPDI_CS_ASI_KEY_STATUS };
const static uint8_t ldcs_sys_stat[2]  = { UPDI_SYNCH, UPDI_LDCS|UPDI_CS_ASI_SYS_STATUS };
bool UPDIUart_Class::START_USERROW (void) {
  BeginTransaction();
  SEND((uint8_t*)urowwrite_key, sizeof(urowwrite_key));
  SEND((uint8_t*)ldcs_key_stat, sizeof(ldcs_key_stat));
  int c = RECV();
  if (c < 0 || (c & 0x20) == 0) return false; // 0x20:UROWWRITE check
  RESET();
  uint32_t ms = millis();
  do {
    BeginTransaction();
    SEND((uint8_t*)ldcs_sys_stat, sizeof(ldcs_sys_stat));
    int c = RECV();
    if ((c >= 0) && (c & 4)) return true;     // 0x04:UROWPROG OK
  } while ((millis() - ms) <= (UPDI_RECV_TIMEOUT << 2));
  return false;
}

const static uint8_t ldcs_asi_ctrla[2] = { UPDI_SYNCH, UPDI_LDCS|UPDI_CS_ASI_CTRLA };
const static uint8_t stcs_sys_ctrla[2] = { UPDI_SYNCH, UPDI_STCS|UPDI_CS_ASI_SYS_CTRLA };
const static uint8_t stcs_key_stat[2]  = { UPDI_SYNCH, UPDI_STCS|UPDI_CS_ASI_KEY_STATUS };
bool UPDIUart_Class::FINAL_USERROW (void) {
  BeginTransaction();               // UPDI_CS_ASI_CTRLA
  SEND((uint8_t*)stcs_sys_ctrla, sizeof(stcs_sys_ctrla));
  SEND(3);                          // 0x02:UROWWRITE_FINAL + 0x01:CLKREQ
  EndTransaction(0);
  uint32_t ms = millis();
  do {
    BeginTransaction();
    SEND((uint8_t*)ldcs_sys_stat, sizeof(ldcs_sys_stat));
    int c = RECV();                 // UPDI_CS_ASI_KEY_STATUS
    if ((c >= 0) && (c & 4) == 0) { // 0x04:UROWPROG END
      BeginTransaction();
      SEND((uint8_t*)stcs_key_stat, sizeof(stcs_key_stat));
      SEND(0x20);                   // 0x20:UROWWRITE
      EndTransaction(0);
      RESET();
      return true;
    }
  } while ((millis() - ms) <= (UPDI_RECV_TIMEOUT << 2));
  return false;
}

bool UPDIUart_Class::ENABLE_NVMPROG (void) {
  BeginTransaction();
  SEND((uint8_t*)ldcs_sys_stat, sizeof(ldcs_sys_stat));
  int c = RECV();
  if ((c >= 0) && (c & 8)) return true;
  BeginTransaction();
  SEND((uint8_t*)nvmprog_key, sizeof(nvmprog_key));
  SEND((uint8_t*)ldcs_key_stat, sizeof(ldcs_key_stat));
  c = RECV();                                   // UPDI_CS_ASI_KEY_STATUS
  if (c < 0 || (c & 0x10) == 0) return false;   // 0x10:NVMPROG check
  RESET();
  uint32_t ms = millis();
  do {
    BeginTransaction();
    SEND((uint8_t*)ldcs_sys_stat, sizeof(ldcs_sys_stat));
    int c = RECV();                             // UPDI_CS_ASI_SYS_STATUS
    if ((c >= 0) && (c & 8)) return true;       // 0x04:NVMPROG OK
  } while ((millis() - ms) <= (UPDI_RECV_TIMEOUT << 4));
  return false;
}

uint8_t UPDIUart_Class::NVM_WAIT (void) {
  while (LD8(NVMCTRL_BASE + 2) & 3);
  return _rx_data;
}

bool UPDIUart_Class::NVM_CMD (uint8_t cmd) {
  return ST8(NVMCTRL_BASE, cmd);
}

bool UPDIUart_Class::ERASE_CHIP (void) {
  RESET();
  BeginTransaction();
  SEND((uint8_t*)erase_key, sizeof(erase_key));
  SEND((uint8_t*)ldcs_key_stat, sizeof(ldcs_key_stat));
  int c = RECV();
  if (c < 0 || (c & 0x08) == 0) return false;
  RESET();
  uint32_t ms = millis();
  do {
    BeginTransaction();
    SEND((uint8_t*)ldcs_sys_stat, sizeof(ldcs_sys_stat));
    int c = RECV();
    if (c < 0) break;
    if (c & 1) return true;
  } while ((millis() - ms) <= (UPDI_RECV_TIMEOUT << 4));
  return false;
}

struct fuse_packet_t { uint16_t data; uint16_t addr; };
bool UPDIUart_Class::ST_FUSE (uint16_t index, uint8_t data) {
  fuse_packet_t fuse_packet;
  fuse_packet.data = data;
  fuse_packet.addr = index;
  NVM_WAIT();
  if (!STS8(NVMCTRL_BASE + 6, (uint8_t*)&fuse_packet, sizeof(fuse_packet))) return false;
  if (!NVM_CMD(NVM_CMD_WFU)) return false;
  return 0 == NVM_WAIT();
}

// end of code
