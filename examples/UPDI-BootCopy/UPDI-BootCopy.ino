/***************
 *
 * UPDI-BootCopy.ino
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

#define FLASH_BASE 0x4000
#define FLASH_PART 128
#define BOOT_PART  4

#if !defined(__AVR_XMEGA__)
#error megaAVR/tinyAVR platform only
#endif

#define BUTTON PIN_PC2

void setup (void) {
  attachInterrupt(PIN_HWSERIAL0_RX, nullptr, CHANGE);
  attachInterrupt(BUTTON, nullptr, CHANGE);
  Serial.begin(CONSOLE_BAUD); delay(200);
  Serial.println("\r\n<<< StartUp >>>");
  Serial.print("F_CPU:"); Serial.println(F_CPU);
}

void loop (void) {
  Serial.println("[Hit CONSOLE or Button to start]");
  Serial.flush();
  power_halt(SLEEP_MODE_PWR_DOWN);
  UPDI.begin(UPDI_BAUD);
  write_bootcode();
  UPDI.end();
}

void write_bootcode (void) {
  int updirev = UPDI.START();
  if (updirev < 0x10) {
    Serial.println("UPDI fail");
    return;
  }
  Serial.print("UPDI revision=0x");
  Serial.println(updirev, HEX);
  if (!UPDI.ERASE_CHIP()) {
    Serial.println("ERASE fail");
    return;
  }
  Serial.println("ERASE ok");
  if (!UPDI.ENABLE_NVMPROG()) {
    Serial.println("NVM fail");
    return;
  }
  for (uint8_t i = 0; i < 11; i++) {
    uint8_t fuse = *((uint8_t*)&FUSE + i);
    if (!UPDI.ST_FUSE((uint16_t)&FUSE + i, fuse)) {
      Serial.println("FUSE fail");
      return;
    }
    if (UPDI.NVM_WAIT()) {
      Serial.println("FUSE fail");
      return;
    }
  }
  Serial.println("FUSE ok");
  uint8_t boot_size = ((uint16_t)FUSE.BOOTEND << 8) / FLASH_PART;

  uint16_t addr = FLASH_BASE;
  for (uint8_t page = 0; page < boot_size; page++) {
    if (FLASH_PART != UPDI.STS8(addr, (uint8_t*)addr, FLASH_PART)) {
      Serial.println("WRITE fail");
      return;
    }
    if (!UPDI.NVM_CMD(NVM_CMD_ERWP)) {
      Serial.println("WRITE fail");
      return;
    }
    if (UPDI.NVM_WAIT()) {
      Serial.println("WRITE fail");
      return;
    }
    addr += FLASH_PART;
  }
  Serial.println("WRITE ok");
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
