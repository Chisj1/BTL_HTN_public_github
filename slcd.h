#ifndef __SLCD_H__
#define __SLCD_H__

#define _LCDRVEN          (1)  
#define _LCDRVTRIM        (8)       // CPSEL = 1     0 -- 8000 pf 1 -- 6000 pf  2 -- 4000 pf  3 -- 2000 pf
#define _LCDCPSEL         (1)       //  charge pump select 0 or 1 
#define _LCDLOADADJUST    (3)       // CPSEL = 1     0 -- 8000 pf 1 -- 6000 pf  2 -- 4000 pf  3 -- 2000 pf
#define _LCDALTDIV        (0)       // CPSEL = 1     0 -- 8000 pf 1 -- 6000 pf  2 -- 4000 pf  3 -- 2000 pf
#define _LCDALRCLKSOURCE  (0)       // 0 -- External clock       1 --  Alternate clock

#define _LCDCLKPSL        (0)       //  Clock divider to generate the LCD Waveforms 
#define _LCDSUPPLY        (1) 
#define _LCDHREF          (0)       // 0 or 1 
#define _LCDCLKSOURCE     (1)       // 0 -- External clock       1 --  Alternate clock
#define _LCDLCK           (1)       //Any number between 0 and 7 
#define _LCDBLINKRATE     (3)       //Any number between 0 and 7 

#define  _LCDFRONTPLANES  (8)       // # of frontPlanes
#define  _LCDBACKPLANES   (4)       // # of backplanes

#define _LCDUSEDPINS   (_LCDFRONTPLANES + _LCDBACKPLANES)
#define _LCDDUTY       (_LCDBACKPLANES-1)         //Any number between 0 and 7 
#define  LCD_WF_BASE    LCD_WF3TO0                                                 

#define _CHARNUM        (4)         //number of chars that can be written
#define _CHAR_SIZE      (2)         // Used only when Dot Matrix is used
#define _LCDTYPE        (2)         //indicate how many LCD_WF are required to write a single Character / or Colum in case of DOT matrix LCD

#define ASCCI_TABLE_START '0'       // indicates which is the first Ascii character in the table
#define ASCCI_TABLE_END   'Z'       // indicates which is the first Ascii character in the table
#define BLANK_CHARACTER   '>'       // Inidicate which ASCII character is a blank character (depends on ASCII table)

#define SEGDP 0x01
#define SEGC  0x02
#define SEGB  0x04
#define SEGA  0x08
                  
#define SEGD  0x01
#define SEGE  0x02
#define SEGG  0x04
#define SEGF  0x08

#define SLCD_Write(type)	SLCD_WriteMsg((unsigned char *)"##type");


// LCD PIN1 to LCDWF0  Rev B
#define CHAR1a      37      // LCD Pin 5
#define CHAR1b      17      // LCD Pin 6
#define CHAR2a      7       // LCD Pin 7
#define CHAR2b      8       // LCD Pin 8
#define CHAR3a      53      // LCD Pin 9
#define CHAR3b      38      // LCD Pin 10
#define CHAR4a      10      // LCD Pin 11
#define CHAR4b      11      // LCD Pin 12
#define CHARCOM0    40      // LCD Pin 1
#define CHARCOM1    52      // LCD Pin 2
#define CHARCOM2    19      // LCD Pin 3
#define CHARCOM3    18      // LCD Pin 4

#define SEGDP       0x01
#define SEGC        0x02
#define SEGB        0x04
#define SEGA        0x08
                    
#define SEGD        0x01
#define SEGE        0x02
#define SEGG        0x04
#define SEGF        0x08

void SLCD_Init(void);
void SLCD_EnablePins(void);
void SLCD_WriteMsg(unsigned char *lbpMessage);
void SLCD_WriteChar(unsigned char lbValue);

#endif
