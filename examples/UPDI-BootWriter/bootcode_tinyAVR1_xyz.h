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
#define SIGROW_BASE  0x1100
#define FUSE_BASE    0x1280
#define USERROW_BASE 0x1300
#define EEPROM_BASE  0x1400
#define SRAM_END     0x4000
#define FLASH_BASE   0x4000

/* Particle sector size */
#define FUSE_PART    11
#define EEPROM_PART  64
#define FLASH_PART   128
#define FLASH_SECTOR 128

/* SIGROW1,2 support list */
struct sigrow_t {
  const uint8_t sigrow0;      // 0x1E only
  const uint8_t sigrow1;      // memory size in series
  const uint8_t sigrow2;      // product
  const uint8_t bootcode;     // BOOTCODE sector size : 0 is disable
  const uint8_t urow_size;    // 0x20 or 0x40
  const uint8_t eeprom_pages; // * eeprom_psize units
  const uint16_t flash_pages; // * flash_psize units
  const uint16_t sram_base;   // x flash_psize
};
const sigrow_t sigrow[] = {
  {0x1e, 0x95, 0x21, 0x02, 0x20, 0x04, 0x0100, 0x3800}, // ATtiny3216
  {0x1e, 0x95, 0x22, 0x02, 0x20, 0x04, 0x0100, 0x3800}  // ATtiny3217
};

/* NVM FUSE */
const uint8_t fuse_idx[] = { 0, 1, 2, 4, 5, 6, 7, 8, 10 };
/* project default       WDT   BOD   OSC   TCD   CFG0  CFG1  APP   BOOT  LOCK */
const uint8_t fuse[] = { 0x00, 0x00, 0x01, 0x00, 0xc5, 0x04, 0x00, 0x02, 0xC5 };

/* optiboot_txyz_1sec.hex */
const char bootcode[] = R"##(
:1000000001C0D8C0112480914000882369F0282FB6
:1000100030E083FD03C02D7F232B31F4809340001B
:100020008CBB80E0AFD0ECC0A895229A2A9A10929F
:100030000102809182128370813061F58CE590E03D
:10004000809308089093090881E080930B0883E06F
:10005000809307081092050880EC8093060888E0DA
:1000600091D0079A87E08150C1F4A8950DE983E00B
:10007000D82E7CD08134E9F479D0182F8CD081E04F
:10008000123821F089E0113809F083E068D080E16E
:1000900066D0EFCF83E790E0D3CF179A2EEC36E50A
:1000A000A8959091040897FDE1CF21503109C1F73F
:1000B000DACF823419F484E176D0E9CF853411F4B3
:1000C00085E0FACF853531F451D0C82F4FD0D82FE5
:1000D00062D0DDCF863521F484E065D080E0D6CFD4
:1000E0008436B9F443D042D0182F40D0863479F406
:1000F000D0583CD0888321961150D9F74CD004BFFA
:10010000D0920010809102108370E1F7C0CFDC5EC6
:10011000F0CF843791F42AD029D0182F27D0F82E89
:100120003AD086E4F81207C0D0588881219617D0BB
:100130001150D9F7ACCFDC5EF8CF853751F42BD016
:10014000809100110CD08091011109D08091021191
:100150009DCF813509F0BCCF81E014D0B9CF90910B
:10016000040895FFFCCF809302080895809104084D
:1001700087FFFCCF909101088091000892FD01C09B
:10018000A89508959091010190FDFCCF98ED94BF42
:10019000809300010895EADF803219F081E0F2DFF8
:1001A000FFCF84E1DCCFCF93C82FE0DFC150E9F768
:1001B000CF91F1CF683048F48DE984BF609300108F
:1001C000809102108370E1F70895FC014083089547
:0201FE000109F5
:00000001FF
)##";

const char eeprom[] = "abcdefghijklmnopqrstuvwxyz012345";
const char userrow[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdef";

// end of code
