/***************
 *
 * bootcode.h
 *
 * release site: https://github.com/askn37/UPDIUart/
 * maintainer: askn https://twitter.com/askn37
 */
/* System Infomation Block */
const char devtype[] = "tinyAVR";

/* Base address */
#define SIGROW_BASE	 0x1100
#define FUSE_BASE    0x1280
#define USERROW_BASE 0x1300
#define EEPROM_BASE  0x1400
#define SRAM_END     0x4000
#define FLASH_BASE   0x4000

/* Particle sector size */
#define FUSE_PART    11
#define EEPROM_PART  32
#define FLASH_PART   64
#define FLASH_SECTOR 128

/* SIGROW1,2 support list */
struct sigrow_t {
  const uint8_t sigrow0;			// 0x1E only
  const uint8_t sigrow1;			// memory size in series
  const uint8_t sigrow2;			// product
  const uint8_t bootcode;			// BOOTCODE sector size : 0 is disable
  const uint8_t urow_size;		// 0x20 or 0x40
  const uint8_t eeprom_pages;	// * eeprom_psize
  const uint16_t flash_pages;	// * flash_psize
  const uint16_t sram_base;		// x flash_psize
};
const sigrow_t sigrow[] = {
  {0x1e, 0x92, 0x23, 0x00, 0x20, 0x04, 0x0040, 0x3f00}	// ATtiny412
};

/* NVM FUSE */
const uint8_t fuse_idx[] = { 0, 1, 2, 4, 5, 6, 7, 8, 10 };
/* project default       WDT   BOD   OSC   TCD   CFG0  CFG1  APP   BOOT  LOCK */
const uint8_t fuse[] = { 0x00, 0x00, 0x01, 0x00, 0xc5, 0x04, 0x00, 0x00, 0xC5 };

/* Optiboot_XXXX.hex */
const char bootcode[] = R"##(
)##";

const char eeprom[] = "abcdefghijklmnopqrstuvwxyz012345";
const char userrow[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdef";

// end of code
