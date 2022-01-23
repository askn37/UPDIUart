/***************
 *
 * bootcode.h
 *
 * release site: https://github.com/askn37/UPDIUart/
 * maintainer: askn https://twitter.com/askn37
 */
/* System Infomation Block */
const char devtype[] = "megaAVR";

/* Base address */
#define SIGROW_BASE	 0x1100
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
  const uint8_t sigrow0;			// 0x1E only
  const uint8_t sigrow1;			// memory size in series
  const uint8_t sigrow2;			// product
  const uint8_t bootcode;			// bootcode write enable
  const uint8_t urow_size;		//
  const uint8_t eeprom_pages;	// x eeprom_psize
  const uint16_t flash_pages;	// x flash_psize
  const uint16_t sram_base;		// x flash_psize
};
const sigrow_t sigrow[] = {
  {0x1e, 0x96, 0x50, 0x02, 0x40, 0x04, 0x0180, 0x2800},	// ATmega4808
  {0x1e, 0x96, 0x51, 0x02, 0x40, 0x04, 0x0180, 0x2800},	// ATmega4809
  {0x1e, 0x95, 0x30, 0x02, 0x40, 0x04, 0x0100, 0x3000},	// ATmega3208
  {0x1e, 0x95, 0x31, 0x02, 0x40, 0x04, 0x0100, 0x3000},	// ATmega3209
  {0x1e, 0x94, 0x27, 0x02, 0x20, 0x04, 0x0080, 0x3800},	// ATmega1608
  {0x1e, 0x94, 0x26, 0x02, 0x20, 0x04, 0x0080, 0x3800},	// ATmega1609
  {0x1e, 0x93, 0x26, 0x02, 0x20, 0x04, 0x0040, 0x3c00},	// ATmega808
  {0x1e, 0x93, 0x2a, 0x02, 0x20, 0x04, 0x0040, 0x3c00}	// ATmega809
};

/* NVM FUSE */
const uint8_t fuse_idx[] = { 0, 1, 2, 5, 6, 7, 8, 10 };
/* project default       WDT   BOD   OSC   CFG0  CFG1  APP   BOOT  LOCK */
const uint8_t fuse[] = { 0x00, 0x54, 0x01, 0xc9, 0x06, 0x00, 0x02, 0xC5 };

/* Optiboot_mega0_UART0_DEF_115200_A7.hex */
const char bootcode[] = R"##(
:1000000001C0D1C01124809140008093400083FF43
:1000100004C0282E80E0AFD0F3C0A895009A089ABB
:100020001092E205809182128370813029F58CE56F
:1000300090E0809308089093090881E080930B0872
:1000400083E0809307081092050880EC80930608EF
:1000500088E091D0079A87E0815089F4A8950DE94E
:1000600083E0D82E7CD08134F1F479D0182F8CD055
:10007000123889F480E013C083E790E0DACF179A52
:100080002EEC36E5A8959091040897FDE8CF215015
:100090003109C1F7E1CF89E0113809F083E058D088
:1000A00080E156D0DFCF823419F484E175D0F8CFE7
:1000B000853411F485E0FACF853531F450D0C82F5E
:1000C0004ED0D82F61D0ECCF863519F484E064D0BF
:1000D000D1CF8436B9F443D042D0182F40D08634E3
:1000E00079F4D05C3CD0888321961150D9F74CD05C
:1000F00004BFD0920010809102108370E1F7D0CF3E
:10010000DC5EF0CF843791F42AD029D0182F27D085
:10011000F82E3AD086E4F81207C0D05C8881219688
:1001200017D01150D9F7BCCFDC5EF8CF853751F42A
:100130002BD0809100110CD08091011109D08091B9
:100140000211ADCF813509F0BDCF81E014D0BACF17
:100150009091040895FFFCCF809302080895809148
:10016000040887FFFCCF909101088091000892FD60
:1001700001C0A89508959091010190FDFCCF98EDE4
:1001800094BF809300010895EADF803219F081E086
:10019000F2DFFFCF84E1DCCFCF93C82FE0DFC15087
:1001A000E9F7CF91F1CF683048F48DE984BF6093CF
:1001B0000010809102108370E1F70895FC014083E4
:0201C0000895A0
:0201FE000009F6
:00000001FF
)##";

const uint8_t eeprom[] = {'a', 'b', 'c', 'd'};
const uint8_t userrow[] = {'A', 'B', 'C', 'D'};

// end of code