/***************
 *
 * bootcode_megaAVR0.h
 *
 * release site: https://github.com/askn37/UPDIUart/
 * maintainer: askn https://twitter.com/askn37
 */
/* System Infomation Block */
const char devtype[] = "megaAVR";

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
#define FLASH_SECTOR 256

/* SIGROW1,2 support list */
struct sigrow_t {
  const uint8_t sigrow0;      // 0x1E only
  const uint8_t sigrow1;      // memory size in series
  const uint8_t sigrow2;      // product
  const uint8_t bootcode;     // bootcode write enable
  const uint8_t urow_size;    //
  const uint8_t eeprom_pages; // x eeprom_psize
  const uint16_t flash_pages; // x flash_psize
  const uint16_t sram_base;   // x flash_psize
};
const sigrow_t sigrow[] = {
  {0x1e, 0x96, 0x50, 0x02, 0x40, 0x04, 0x0180, 0x2800}, // ATmega4808
  {0x1e, 0x96, 0x51, 0x02, 0x40, 0x04, 0x0180, 0x2800}, // ATmega4809
  {0x1e, 0x95, 0x30, 0x02, 0x40, 0x04, 0x0100, 0x3000}, // ATmega3208
  {0x1e, 0x95, 0x31, 0x02, 0x40, 0x04, 0x0100, 0x3000}, // ATmega3209
  {0x1e, 0x94, 0x27, 0x02, 0x20, 0x04, 0x0080, 0x3800}, // ATmega1608
  {0x1e, 0x94, 0x26, 0x02, 0x20, 0x04, 0x0080, 0x3800}, // ATmega1609
  {0x1e, 0x93, 0x26, 0x02, 0x20, 0x04, 0x0040, 0x3c00}, // ATmega808
  {0x1e, 0x93, 0x2a, 0x02, 0x20, 0x04, 0x0040, 0x3c00}  // ATmega809
};

/* NVM FUSE */
const uint8_t fuse_idx[] = { 0, 1, 2, 5, 6, 7, 8, 10 };
/* project default       WDT   BOD   OSC   CFG0  CFG1  APP   BOOT  LOCK */
const uint8_t fuse[] = { 0x00, 0x54, 0x01, 0xc9, 0x06, 0x00, 0x02, 0xC5 };

/* boot_atmega_UART0_A0_115200_LA7.hex */
const char bootcode[] = R"##(
:100000002DC009C09DE994BF80930010809102101B
:100010008370E1F70895FC016083089590910408CE
:1000200095FFFCCF8093020808958091040887FF14
:10003000FCCF909101088091000892FFA895179A33
:100040000895F3DF803209F0DBCF84E1E7CFCF936F
:10005000C82FEBDFC150E9F7CF91F3CF1124809186
:100060004000809340008CBB811105C098ED21E0D9
:1000700094BF20934100E39902C0843008F4C0C0CB
:10008000079A8091821280FF2CC08CE5809308082B
:1000900083E08093070880EC8093060886E0179A37
:1000A00028E93AE39091040897FD05C021503109F1
:1000B000C9F78150A1F788ED98E084BF90930001C3
:1000C00083E0D82EB2DF813491F4AFDFF82EB9DFB0
:1000D00082E8F81641F081E8F81207C083E09EDF5D
:1000E0000AC083E7D3CF87E0FACF80E0F8CF82342D
:1000F00029F484E1ACDF80E191DFE4CF853411F4B1
:1001000085E0F8CF853531F490DFC82F8EDFD82F0A
:1001100098DFF1CF8436C9F488DF182F86DF082FE7
:1001200084DF863481F4D05C809102108370E1F723
:100130007CDF8883219601501109D1F78DE984BFB6
:10014000D0920010E5CFDC5EEFCF843791F46DDF05
:10015000182F6BDF082F69DF863449F4D05C71DF1C
:10016000888121965BDF01501109D1F7C4CFDC5E95
:10017000F6CF853749F465DF8EE150DF80910111BC
:0C0180004DDF80910211ABCF009AC2CF7E
:00000001FF
)##";

const uint8_t eeprom[] = {'a', 'b', 'c', 'd'};
const uint8_t userrow[] = {'A', 'B', 'C', 'D'};

// end of code
