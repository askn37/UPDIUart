/***************
 *
 * UPDI-BootWriter.ino
 *
 * release site: https://github.com/askn37/UPDIUart/
 * maintainer: askn https://twitter.com/askn37
 */
#include <Arduino.h>
#include <string.h>
#include <ctype.h>
#include <UPDIUart.h>
UPDIUart_Class UPDI(&USART1, PIN_HWSERIAL1_TX, false);

#include "bootcode_megaAVR0.h"      // megaAVR ATmega8/16/32/48,08/09
// #include "bootcode_tinyAVR_8pin.h"  // tinyAVR-0/1 8pin only
// #include "bootcode_tinyAVR_14pin.h" // tinyAVR-0/1/2 14/20/24pin

#ifndef CONSOLE_BAUD
#define CONSOLE_BAUD 9600
#endif

#define UPDI_BAUD 225000L
// #define UPDI_BAUD 230400L

#if !defined(__AVR_XMEGA__)
#error megaAVR/tinyAVR platform only
#endif

sigrow_t *signature = (sigrow_t*)&sigrow[0];  // default signature

void setup (void) {
  Serial.begin(CONSOLE_BAUD); delay(200);
  Serial.println("\r\n<<< StartUp >>>");
  Serial.print("F_CPU:"); Serial.println(F_CPU);
}

void loop (void) {
  Serial.println("==UPDI STATUS==");
  int result = updi_check();
  while (Serial.available()) Serial.read();
  if (result >= 1) {
    Serial.println("[Select menu nummber and enter]");
    Serial.println(" 1:Erase chip and unlocked");
    if (result >= 2) Serial.println(" 2:Write FUSE");
    if (result >= 3) Serial.println(" 3:Write Bootloader");
    if (result >= 3) Serial.println(" 4:Erase+Reset+Write Run");
    if (result >= 2) {
      Serial.println(" 5:Dump Flash");
      Serial.println(" 6:Dump EEPROM");
      Serial.println(" 7:Dump USERROW");
      Serial.println(" L:Write Lock device");
      Serial.println(" E:Write EEPROM");
    }
    Serial.println(" U:Write USERROW");
    Serial.println("[Other:Reload UPDI STATUS]");
    while (!Serial.available());  // key in wait
    char c = Serial.read();
    Serial.print("Selcted:"); Serial.println(c);
    while (Serial.available()) Serial.read();
    UPDI.begin(UPDI_BAUD);
    if (UPDI.START() < 0x10) {
      Serial.println("NVM access failed");
      return;
    }
    switch (c) {
      case '1' : erase_flash(); break;
      case '2' : write_fuse(); break;
      case '3' : write_flash(); break;
      case '4' : erase_flash(); write_fuse(); write_flash(); break;
      case '5' : dump_flash(); break;
      case '6' : dump_eeprom(); break;
      case '7' : dump_userrow(); break;
      case 'E' : write_eeprom(); dump_eeprom(); break;
      case 'L' : lock_device(); break;
      case 'U' : write_userrow(); if (result >= 3) dump_userrow(); break;
    }
    UPDI.end();
  }
  else {
    while (!Serial.available());
  }
  Serial.flush();
}

int updi_check (void) {
  int result = 0;
  uint8_t *buffer;
  size_t len;
  UPDI.begin(UPDI_BAUD);
  for (;;) {
    int updirev = UPDI.START();
    if (updirev < 0x10) {
      Serial.println("UPDI not response");
      break;
    }
    Serial.print("UPDI ok revision=0x");
    Serial.println(updirev, HEX);
    buffer = UPDI.buffer();
    len = UPDI.LDSIB();
    Serial.print("Target system infomation: ");
    Serial.write((char*)buffer, 16);
    Serial.println();
    if (0 != strncmp(devtype, (const char*)buffer, 7)) {
      Serial.print("Undefined SIB not ");
      Serial.println(devtype);
      result = 1;
    }
    if (!UPDI.ENABLE_NVMPROG()) {
      Serial.println("NVMPROG Disable");
      result = 1; break;
    }
    if (!result) fuse_dump();
    len = UPDI.LDS8(SIGROW_BASE, 3);
    Serial.print("SIGROW.DEVICEIDn: ");
    hexdump(buffer, len, false);
    signature = sigrow_check(buffer);
    if (signature == nullptr) {
      Serial.println("Undefined DEVICEIDn");
      result = 1; break;
    }
    if (signature->bootcode) result = 3;
    else result = 2;
    break;
  }
  UPDI.end();
  return result;
}

void addrprint (uint16_t addr) {
  if (addr < 0x10) Serial.write('0');
  if (addr < 0x100) Serial.write('0');
  if (addr < 0x1000) Serial.write('0');
  Serial.print(addr, HEX);
}

void hexdump (uint8_t* data, size_t len, bool ascii) {
  uint8_t *p = data;
  size_t s = len;
  while (s--) {
    uint8_t c = *p++;
    if (c < 0x10) Serial.write('0');
    Serial.print(c, HEX);
    Serial.write(' ');
  }
  if (ascii) {
    for (size_t i = len; i < 16; i++) Serial.print("   ");
    while (len--) {
      uint8_t c = *data++;
      if (!isgraph(c)) c = '.';
      Serial.write(c);
    }
  }
  Serial.println();
}

sigrow_t *sigrow_check (uint8_t* data) {
  for (uint8_t i = 0; i < sizeof(sigrow) / sizeof(sigrow_t); i++) {
    if (data[0] == sigrow[i].sigrow0
     && data[1] == sigrow[i].sigrow1
     && data[2] == sigrow[i].sigrow2) return (sigrow_t*)&sigrow[i];
  }
  return nullptr;
}

void fuse_dump (void) {
  uint8_t* buffer = UPDI.buffer();
  UPDI.LDS8(FUSE_BASE, FUSE_PART);
  Serial.print("FUSE");
  for (auto i : fuse_idx) {
    Serial.write(':');
    uint8_t c = buffer[i];
    if (c < 16) Serial.write('0');
    Serial.print(c, HEX);
  }
  Serial.println();
}

void dump_flash (void) {
  if (!UPDI.ENABLE_NVMPROG()) return;
  uint8_t* buffer = UPDI.buffer();
  size_t len = UPDI.LDS8(SIGROW_BASE, 3);
  signature = sigrow_check(buffer);
  if (signature == nullptr) return;
  Serial.println("==DUMP FLASH==");
  Serial.println("[q:quit]");
  int before = -1;
  uint8_t store[FLASH_SECTOR];
  bool check = false;
  for (uint16_t page = 0; page < (signature->flash_pages >> 1); page++) {
    if (page) {
      if (!check) {
        while (Serial.available()) Serial.read();
        while (!Serial.available());
      }
      int c = Serial.read();
      if (c == 'q') break;
    }
    uint16_t addr = page << 8;
    len = UPDI.LDS8(FLASH_BASE + addr, store, FLASH_SECTOR);
    int repeat = store[0];
    check = true;
    for (size_t i = 0; i < len; i++) {
      if (store[i] != repeat) repeat = -1;
      if (store[i] != before) check = false;
    }
    if (check) continue;
    if (repeat != -1) {
      addrprint(addr);
      Serial.print(" repeat ");
      before = repeat;
      if (before < 16) Serial.write('0');
      Serial.println(before, HEX);
      check = true;
      continue;
    }
    dump_page(addr, store, FLASH_SECTOR);
    Serial.println("----");
  }
  Serial.print("END address:");
  Serial.println((signature->flash_pages << 7) - 1, HEX);
}

void dump_userrow (void) {
  if (!UPDI.ENABLE_NVMPROG()) return;
  uint8_t* buffer = UPDI.buffer();
  size_t len = UPDI.LDS8(SIGROW_BASE, 3);
  signature = sigrow_check(buffer);
  if (signature == nullptr) return;
  Serial.println("==DUMP USERROW==");
  uint8_t store[EEPROM_PART];
  len = UPDI.LDS8(USERROW_BASE, store, signature->urow_size);
  dump_page(USERROW_BASE, store, len);
}

void dump_eeprom (void) {
  if (!UPDI.ENABLE_NVMPROG()) return;
  uint8_t* buffer = UPDI.buffer();
  size_t len = UPDI.LDS8(SIGROW_BASE, 3);
  signature = sigrow_check(buffer);
  if (signature == nullptr) return;
  Serial.println("==DUMP EEPROM==");
  uint8_t store[EEPROM_PART];
  uint16_t offset = 0;
  for (uint8_t page = 0; page < signature->eeprom_pages; page++) {
    uint16_t addr = EEPROM_BASE + page * EEPROM_PART;
    len = UPDI.LDS8(addr, store, EEPROM_PART);
    dump_page(addr, store, len);
  }
}

void dump_page (uint16_t addr, uint8_t *data, size_t len) {
  size_t count = 0;
  size_t size = 16;
  while (count < len) {
    addrprint(addr);
    addr += size;
    count += size;
    Serial.write(' ');
    if (count > len) size = count - len;
    hexdump(data, size, true);
    data += size;
  }
}

void lock_device (void) {
  Serial.println("==LOCK DEVICE==");
  if (!UPDI.ENABLE_NVMPROG()) {
    Serial.println("NVM failed");
    return;
  }
  if (!UPDI.ST_FUSE(FUSE_BASE + FUSE_PART - 1, 0)) {
    Serial.println("Write FUSE failed");
    return;
  }
  Serial.println("-Complete-");
}

void erase_flash (void) {
  Serial.println("==CHIP ERASE==");
  if (!UPDI.ERASE_CHIP()) {
    Serial.println("Erase failed");
    return;
  }
  Serial.println("ERASE ok");
  if (!UPDI.ENABLE_NVMPROG()) {
    Serial.println("NVM failed");
    return;
  }
  if (!UPDI.ST_FUSE(FUSE_BASE + 0x0a, 0xc5)) {
    Serial.println("Write FUSE failed");
    return;
  }
  UPDI.NVM_WAIT();
  Serial.println("-Complete-");
}

void write_fuse (void) {
  if (!UPDI.ENABLE_NVMPROG()) return;
  uint8_t* buffer = UPDI.buffer();
  size_t len = UPDI.LDS8(SIGROW_BASE, 3);
  Serial.print("SIGROW.DEVICEIDx: "); hexdump(buffer, len, false);
  signature = sigrow_check(buffer);
  if (signature == nullptr) {
    Serial.println("Undefined DEVICEIDx");
    return;
  }
  Serial.println("==WRITE FUSE==");
  for (uint8_t i = 0; i < sizeof(fuse_idx); i++) {
    if (!UPDI.ST_FUSE(FUSE_BASE + fuse_idx[i], fuse[i])) {
      Serial.println("Write FUSE failed");
      return;
    }
  }
  UPDI.NVM_WAIT();
  Serial.println("-Complete-");
}

void write_flash (void) {
  if (!UPDI.ENABLE_NVMPROG()) return;
  uint8_t* buffer = UPDI.buffer();
  size_t len = UPDI.LDS8(SIGROW_BASE, 3);
  signature = sigrow_check(buffer);
  if (signature == nullptr || !signature->bootcode) return;
  Serial.println("==WRITE FLASH==");
  boot_flush(signature->flash_pages);
  Serial.println("-Complete-");
}

void boot_flush (uint16_t flash_pages) {
  char *ptr, a, b;
  uint8_t sum, duty = 0, store[FLASH_PART], buffer[23];
  uint16_t addr, page = 0, count = 0;
  size_t len;
  ptr = (char*) bootcode;
  for (uint8_t i = 0; i < FLASH_PART; i++) store[i] = ~0;
  for (;;) {
    ptr = strchr(ptr, ':');
    if (ptr == nullptr) break;
    ptr++; len = sum = 0;
    for (;;) {
      a = *ptr++; if (!isxdigit(a)) break;
      b = *ptr++; if (!isxdigit(b)) break;
      sum += buffer[len++] = (atox(a) << 4) | atox(b);
    }
    if (sum != 0 || (len - 5 != buffer[0])) continue;
    if (buffer[3] != 0) break;
    len -= 5;
    addr = ((uint16_t)buffer[1] << 8) | buffer[2];
    memcpy(store + (addr & (FLASH_PART-1)), buffer + 4, len);
    duty = 1;
    uint16_t next = ((addr + len) / FLASH_PART);
    if (next != page) {
      addrprint(page * FLASH_PART); Serial.write('.');
      if ((count++ & 7) == 7) Serial.println();
      UPDI.STS8(FLASH_BASE + (page * FLASH_PART), store, FLASH_PART);
      UPDI.NVM_CMD(NVM_CMD_ERWP);
      UPDI.NVM_WAIT();
      page = next;
      if (page >= flash_pages) break;
      for (uint8_t i = 0; i < FLASH_PART; i++) store[i] = ~0;
      duty = 0;
      /* page overflow */
      int16_t ovf = (addr & (FLASH_PART - 1)) + len - FLASH_PART;
      if (0 < ovf) {
        memcpy(store, buffer + 4 + len - ovf, ovf);
        duty = 1;
      }
    }
  }
  if (duty) {
    addrprint(page * FLASH_PART);
    UPDI.STS8(FLASH_BASE + (page * FLASH_PART), store, FLASH_PART);
    UPDI.NVM_CMD(NVM_CMD_ERWP);
    UPDI.NVM_WAIT();
  }
  UPDI.NVM_WAIT();
  Serial.println();
}

uint8_t atox (char a) {
  a |= 0x20;
  a -= 0x30;
  if (a >= 10) a -= 0x27;
  return a & 15;
}

void write_userrow (void) {
  if (!signature->urow_size) {
    Serial.println("signature not settings");
    return;
  }
  if (!UPDI.START_USERROW()) {
    Serial.println("USERROW start failed");
    return;
  }
  Serial.println("==WRITE USERROW==");
  size_t len = min(signature->urow_size, sizeof(userrow));
  uint8_t store[64];
  for (uint8_t i = 0; i < signature->urow_size; i++) store[i] = ~0;
  memcpy(store, userrow, len);
  UPDI.STS8(USERROW_BASE, store, signature->urow_size);
  if (!UPDI.FINAL_USERROW()) {
    Serial.println("USERROW final failed");
    return;
  }
  Serial.println("-Complete-");
}

void write_eeprom (void) {
  if (!UPDI.ENABLE_NVMPROG()) return;
  uint8_t* buffer = UPDI.buffer();
  size_t len = UPDI.LDS8(SIGROW_BASE, 3);
  Serial.print("SIGROW.DEVICEIDx: "); hexdump(buffer, len, false);
  signature = sigrow_check(buffer);
  if (signature == nullptr) {
    Serial.println("Undefined DEVICEIDx");
    return;
  }
  Serial.println("==WRITE EEPROM==");
  uint8_t store[EEPROM_PART];
  uint16_t offset = 0;
  for (uint8_t page = 0; page < signature->eeprom_pages; page++) {
    for (uint8_t i = 0; i < EEPROM_PART; i++) store[i] = ~0;
    uint16_t addr = EEPROM_BASE + page * EEPROM_PART;
    len = min(sizeof(eeprom) - offset, EEPROM_PART);
    memcpy(store, &userrow[offset], len);
    UPDI.STS8(addr, store, EEPROM_PART);
    UPDI.NVM_CMD(NVM_CMD_ERWP);
    UPDI.NVM_WAIT();
    if (len < EEPROM_PART) break;
    offset += EEPROM_PART;
  }
  UPDI.NVM_WAIT();
  Serial.println("-Complete-");
}

// end of code
