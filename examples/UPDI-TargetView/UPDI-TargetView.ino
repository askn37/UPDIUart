/***************
 *
 * UPDI-TargetView.ino
 *
 * release site: https://github.com/askn37/UPDIUart/
 * maintainer: askn https://twitter.com/askn37
 */
#include <Arduino.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <UPDIUart.h>
UPDIUart_Class UPDI(&USART1, PIN_HWSERIAL1_TX, false);

#ifndef CONSOLE_BAUD
#define CONSOLE_BAUD 9600
#endif

#define UPDI_BAUD 225000L
// #define UPDI_BAUD 230400L

#if !defined(__AVR_XMEGA__)
#error megaAVR/tinyAVR platform only
#endif

struct dev_type_t {
  uint8_t sernum0;	// SERNUM0 base addr
  uint8_t sernumx;	// SERNUMx length
  uint8_t fuse;			// Disable CRC, Enable UPDI
};

dev_type_t dev_type[] = {
  { 0x03, 0x0a, 0xc5 },
  { 0x03, 0x0a, 0xc9 },
  { 0x10, 0x10, 0xc9 }
};

enum dev_type_e { TINY, MEGA, AVRD };

struct updi_target_t {
  uint8_t updirev;
  uint8_t cfg_type;
  uint8_t devid1;		/* devid0 == 0x1e */
  uint8_t devid2;
  const char *devname;
};

updi_target_t updi_target_list[] = {
  { 0x20, TINY, 0x91, 0x23, "ATtiny202"  },
  { 0x20, TINY, 0x91, 0x22, "ATtiny204"  },
  { 0x20, TINY, 0x92, 0x27, "ATtiny402"  },
  { 0x20, TINY, 0x92, 0x26, "ATtiny404"  },
  { 0x20, TINY, 0x92, 0x25, "ATtiny406"  },

  { 0x20, TINY, 0x93, 0x25, "ATtiny804"  },
  { 0x20, TINY, 0x93, 0x24, "ATtiny806"  },
  { 0x20, TINY, 0x93, 0x23, "ATtiny807"  },
  { 0x20, TINY, 0x94, 0x25, "ATtiny1604" },
  { 0x20, TINY, 0x94, 0x24, "ATtiny1606" },
  { 0x20, TINY, 0x94, 0x23, "ATtiny1607" },

  { 0x20, TINY, 0x91, 0x21, "ATtiny212"  },
  { 0x20, TINY, 0x91, 0x20, "ATtiny214"  },
  { 0x20, TINY, 0x92, 0x23, "ATtiny412"  },
  { 0x20, TINY, 0x92, 0x22, "ATtiny414"  },
  { 0x20, TINY, 0x92, 0x21, "ATtiny416"  },
  { 0x20, TINY, 0x92, 0x20, "ATtiny417"  },

  { 0x20, TINY, 0x93, 0x22, "ATtiny814"  },
  { 0x20, TINY, 0x93, 0x21, "ATtiny816"  },
  { 0x20, TINY, 0x93, 0x20, "ATtiny817"  },
  { 0x10, TINY, 0x94, 0x22, "ATtiny1614" },
  { 0x10, TINY, 0x94, 0x21, "ATtiny1616" },
  { 0x10, TINY, 0x94, 0x20, "ATtiny1617" },
  { 0x10, TINY, 0x95, 0x21, "ATtiny3216" },
  { 0x10, TINY, 0x95, 0x22, "ATtiny3217" },

  { 0x40, TINY, 0x92, 0x2c, "ATtiny424"  },
  { 0x40, TINY, 0x92, 0x2b, "ATtiny426"  },
  { 0x40, TINY, 0x92, 0x2a, "ATtiny427"  },
  { 0x40, TINY, 0x93, 0x29, "ATtiny824"  },
  { 0x40, TINY, 0x93, 0x28, "ATtiny826"  },
  { 0x40, TINY, 0x93, 0x27, "ATtiny827"  },
  { 0x10, TINY, 0x94, 0x28, "ATtiny1624" },
  { 0x10, TINY, 0x94, 0x29, "ATtiny1626" },
  { 0x10, TINY, 0x94, 0x2a, "ATtiny1627" },

  { 0x30, MEGA, 0x93, 0x26, "ATmega808"  },
  { 0x30, MEGA, 0x93, 0x2a, "ATmega809"  },
  { 0x30, MEGA, 0x94, 0x27, "ATmega1608" },
  { 0x30, MEGA, 0x94, 0x26, "ATmega1609" },
  { 0x30, MEGA, 0x95, 0x30, "ATmega3208" },
  { 0x30, MEGA, 0x95, 0x31, "ATmega3209" },
  { 0x30, MEGA, 0x96, 0x50, "ATmega4808" },
  { 0x30, MEGA, 0x96, 0x51, "ATmega4809" },

  { 0x10, AVRD, 0x95, 0x34, "AVR32DA28"  },
  { 0x10, AVRD, 0x95, 0x33, "AVR32DA32"  },
  { 0x10, AVRD, 0x95, 0x32, "AVR32DA48"  },
  { 0x10, AVRD, 0x96, 0x15, "AVR64DA28"  },
  { 0x10, AVRD, 0x96, 0x14, "AVR64DA32"  },
  { 0x10, AVRD, 0x96, 0x13, "AVR64DA48"  },
  { 0x10, AVRD, 0x96, 0x12, "AVR64DA64"  },
  { 0x10, AVRD, 0x97, 0x17, "AVR128DA64" },
  { 0x10, AVRD, 0x97, 0x18, "AVR128DA48" },
  { 0x10, AVRD, 0x97, 0x19, "AVR128DA32" },
  { 0x10, AVRD, 0x97, 0x1a, "AVR128DA28" },

  { 0x30, AVRD, 0x95, 0x38, "AVR32DB28"  },
  { 0x30, AVRD, 0x95, 0x37, "AVR32DB32"  },
  { 0x30, AVRD, 0x95, 0x36, "AVR32DB48"  },
  { 0x30, AVRD, 0x96, 0x19, "AVR64DB28"  },
  { 0x30, AVRD, 0x96, 0x18, "AVR64DB32"  },
  { 0x30, AVRD, 0x96, 0x17, "AVR64DB48"  },
  { 0x30, AVRD, 0x96, 0x16, "AVR64DB64"  },
  { 0x30, AVRD, 0x97, 0x0e, "AVR128DB28" },
  { 0x30, AVRD, 0x97, 0x0d, "AVR128DB32" },
  { 0x30, AVRD, 0x97, 0x0c, "AVR128DB48" },
  { 0x30, AVRD, 0x97, 0x0b, "AVR128DB64" }
};

void setup (void) {
  attachInterrupt(PIN_HWSERIAL0_RX, nullptr, CHANGE);

  Serial.begin(CONSOLE_BAUD); delay(200);
  Serial.println("\r\n<<< StartUp >>>");
  Serial.print("F_CPU:"); Serial.println(F_CPU);
  Serial.flush();
}

void loop (void) {
  UPDI.begin(UPDI_BAUD);
  updi_test();
  UPDI.end();
  Serial.println("-END-");
  Serial.flush();
  power_halt(SLEEP_MODE_PWR_DOWN);
}

void updi_test (void) {
  int updirev = UPDI.START();
  if (updirev < 0x10) {
    Serial.println("UPDI not response");
    return;
  }
  Serial.print("UPDI revision=0x");
  Serial.println(updirev, HEX);
  if (!UPDI.ENABLE_NVMPROG()) {
    Serial.println("NVMPROG Disable");
    return;
  }
  uint8_t* buffer = UPDI.buffer();
  size_t len;
  int c;

  len = UPDI.LDSIB();
  Serial.print("SIB: "); hexdump(buffer, len);
  Serial.print(" Typ: "); Serial.write((char*)buffer, 7); Serial.println();
  Serial.print(" NVM: "); Serial.write((char*)buffer+8, 3); Serial.println();
  Serial.print(" OCD: "); Serial.write((char*)buffer+11, 3); Serial.println();
  Serial.print(" DBG: "); Serial.write((char)buffer[15]); Serial.println();
  UPDI.BeginTransaction();
  UPDI.SEND(UPDI_SYNCH);
  UPDI.SEND(UPDI_LDCS|UPDI_CS_ASI_SYS_STATUS);
  c = UPDI.RECV();
  Serial.print("LDCS_ASI_SYS_STAT=0x"); Serial.println(c, HEX);
  Serial.print(" RSTSYS  : "); Serial.println(c & 0x20 ? "ON" : "OFF");
  Serial.print(" INSLEEP : "); Serial.println(c & 0x10 ? "ON" : "OFF");
  Serial.print(" NVMPROG : "); Serial.println(c & 0x08 ? "ON" : "OFF");
  Serial.print(" UROWPROG: "); Serial.println(c & 0x04 ? "ON" : "OFF");
  Serial.print(" NVMLOCK : "); Serial.println(c & 0x01 ? "ON" : "OFF");
  len = UPDI.LDS8((uint16_t)&SIGROW.DEVICEID0, 3);
  if (len != 3 || buffer[0] != 0x1e) {
    Serial.println("Target NVM load failed (powerdown sleep or NVM locked?)");
    return;
  }
  Serial.print("SIGROW.DEVICEID: ");
  Serial.print("FUSE: "); hexdump(buffer, len);

  /* find in updi_target_list */
  updi_target_t target = {};
  for (auto test : updi_target_list) {
    if (buffer[1] == test.devid1 && buffer[2] == test.devid2) target = test;
  }
  if (!target.updirev) {
    Serial.println("Undefined target device");
    return;
  }
  Serial.print("Find target device : ");
  Serial.println(target.devname);
  dev_type_t device = dev_type[target.cfg_type];

  len = UPDI.LDS8((uint16_t)&SIGROW + device.sernum0, device.sernumx);
  Serial.print("SIGROW.SERNUM: "); hexdump(buffer, len);

  if (target.cfg_type == MEGA) {
    len = UPDI.LDS8((uint16_t)&FUSE, 11);
    Serial.print("FUSE: "); hexdump(buffer, len);
  }
}

void hexdump (uint8_t* data, size_t len) {
  uint8_t *p = data;
  size_t s = len;
  while (s--) {
    uint8_t c = *p++;
    if (c < 0x10) Serial.write('0');
    Serial.print(c, HEX);
    Serial.write(' ');
  }
  Serial.println();
}

#ifndef ZINNIA_UTILS
void power_halt (const uint8_t mode) {
  set_sleep_mode(mode);
  sleep_enable();
  interrupts();
  sleep_cpu();
  sleep_disable();
}
#endif

// end of code
