#include "MKL46Z4.h"
#include "slcd.h"

const unsigned long int MASK_BIT[32] =
{
    0x00000001 ,
    0x00000002 ,
    0x00000004 ,
    0x00000008 ,
    0x00000010 ,
    0x00000020 ,
    0x00000040 ,
    0x00000080 ,
    0x00000100 ,
    0x00000200 ,
    0x00000400 ,
    0x00000800 ,
    0x00001000 ,
    0x00002000 ,
    0x00004000 ,
    0x00008000 ,
    0x00010000 ,
    0x00020000 ,
    0x00040000 ,
    0x00080000 ,
    0x00100000 ,
    0x00200000 ,
    0x00400000 ,
    0x00800000 ,
    0x01000000 ,
    0x02000000 ,
    0x04000000 ,
    0x08000000 ,
    0x10000000 ,
    0x20000000 ,
    0x40000000 ,
    0x80000000 ,
};

const unsigned char WF_ORDERING_TABLE[ ] =
{
    CHAR1a,     // LCD81 --- Pin:5   LCDnAddress=51
    CHAR1b,     // LCD82 --- Pin:6   LCDnAddress=52
    CHAR2a,     // LCD83 --- Pin:7   LCDnAddress=53
    CHAR2b,     // LCD84 --- Pin:8   LCDnAddress=54
    CHAR3a,     // LCD85 --- Pin:9   LCDnAddress=55
    CHAR3b,     // LCD86 --- Pin:10  LCDnAddress=56
    CHAR4a,     // LCD87 --- Pin:11  LCDnAddress=57
    CHAR4b,     // LCD88 --- Pin:12  LCDnAddress=58
    CHARCOM0,   // LCD77 --- Pin:1   LCDnAddress=4D
    CHARCOM1,   // LCD78 --- Pin:2   LCDnAddress=4E
    CHARCOM2,   // LCD79 --- Pin:3   LCDnAddress=4F
    CHARCOM3,   // LCD80 --- Pin:4   LCDnAddress=50

};

/*
   Ascii to 8x6 dot matrix decodification table
*/
const char ASCII_TO_WF_CODIFICATION_TABLE [ ] =
{
( SEGD+ SEGE+ SEGF+!SEGG) , ( SEGC+ SEGB+ SEGA) ,//Char = 0,   offset=0
(!SEGD+!SEGE+!SEGF+!SEGG) , ( SEGC+ SEGB+!SEGA) ,//Char = 1,   offset=4
( SEGD+ SEGE+!SEGF+ SEGG) , (!SEGC+ SEGB+ SEGA) ,//Char = 2,   offset=8
( SEGD+!SEGE+!SEGF+ SEGG) , ( SEGC+ SEGB+ SEGA) ,//Char = 3,   offset=12
(!SEGD+!SEGE+ SEGF+ SEGG) , ( SEGC+ SEGB+!SEGA) ,//Char = 4,   offset=16
( SEGD+!SEGE+ SEGF+ SEGG) , ( SEGC+!SEGB+ SEGA) ,//Char = 5,   offset=20
( SEGD+ SEGE+ SEGF+ SEGG) , ( SEGC+!SEGB+ SEGA) ,//Char = 6,   offset=24
(!SEGD+!SEGE+!SEGF+!SEGG) , ( SEGC+ SEGB+ SEGA) ,//Char = 7,   offset=28
( SEGD+ SEGE+ SEGF+ SEGG) , ( SEGC+ SEGB+ SEGA) ,//Char = 8,   offset=32
( SEGD+!SEGE+ SEGF+ SEGG) , ( SEGC+ SEGB+ SEGA) ,//Char = 9,   offset=36
(!SEGD+!SEGE+!SEGF+!SEGG) , (!SEGC+!SEGB+!SEGA) ,//Char = :,   offset=40
(!SEGD+!SEGE+!SEGF+!SEGG) , (!SEGC+!SEGB+!SEGA) ,//Char = ;,   offset=44
(!SEGD+!SEGE+!SEGF+!SEGG) , (!SEGC+!SEGB+!SEGA) ,//Char = <,   offset=48
( SEGD+!SEGE+!SEGF+ SEGG) , (!SEGC+!SEGB+!SEGA) ,//Char = =,   offset=52
(!SEGD+!SEGE+!SEGF+!SEGG) , (!SEGC+!SEGB+!SEGA) ,//Char = >,   offset=56
(!SEGD+!SEGE+!SEGF+!SEGG) , (!SEGC+!SEGB+ SEGA) ,//Char = ?,   offset=60
( SEGD+ SEGE+ SEGF+ SEGG) , ( SEGC+ SEGB+ SEGA) ,//Char = @,   offset=64
(!SEGD+ SEGE+ SEGF+ SEGG) , ( SEGC+ SEGB+ SEGA) ,//Char = A,   offset=68
( SEGD+ SEGE+ SEGF+ SEGG) , ( SEGC+!SEGB+!SEGA) ,//Char = B,   offset=72
( SEGD+ SEGE+ SEGF+!SEGG) , (!SEGC+!SEGB+ SEGA) ,//Char = C,   offset=76
( SEGD+ SEGE+!SEGF+ SEGG) , ( SEGC+ SEGB+ SEGA) ,//Char = D,   offset=80
( SEGD+ SEGE+ SEGF+ SEGG) , (!SEGC+!SEGB+ SEGA) ,//Char = E,   offset=84
(!SEGD+ SEGE+ SEGF+ SEGG) , (!SEGC+!SEGB+ SEGA) ,//Char = F,   offset=88
( SEGD+ SEGE+ SEGF+ SEGG) , ( SEGC+!SEGB+ SEGA) ,//Char = G,   offset=92
(!SEGD+ SEGE+ SEGF+ SEGG) , ( SEGC+ SEGB+!SEGA) ,//Char = H,   offset=96
(!SEGD+!SEGE+!SEGF+!SEGG) , ( SEGC+!SEGB+!SEGA) ,//Char = I,   offset=100
( SEGD+ SEGE+!SEGF+!SEGG) , ( SEGC+ SEGB+!SEGA) ,//Char = J,   offset=104
(!SEGD+!SEGE+!SEGF+!SEGG) , (!SEGC+!SEGB+!SEGA) ,//Char = K,   offset=108
( SEGD+ SEGE+ SEGF+!SEGG) , (!SEGC+!SEGB+!SEGA) ,//Char = L,   offset=112
(!SEGD+ SEGE+ SEGF+!SEGG) , ( SEGC+ SEGB+!SEGA) ,//Char = M,   offset=116
(!SEGD+ SEGE+!SEGF+ SEGG) , ( SEGC+!SEGB+!SEGA) ,//Char = N,   offset=120
( SEGD+ SEGE+!SEGF+ SEGG) , ( SEGC+!SEGB+!SEGA) ,//Char = O,   offset=124
(!SEGD+ SEGE+ SEGF+ SEGG) , (!SEGC+ SEGB+ SEGA) ,//Char = P,   offset=128
( SEGD+!SEGE+ SEGF+ SEGG) , ( SEGC+ SEGB+ SEGA) ,//Char = Q,   offset=132
(!SEGD+ SEGE+ SEGF+ SEGG) , ( SEGC+ SEGB+ SEGA) ,//Char = R,   offset=136
( SEGD+!SEGE+ SEGF+ SEGG) , ( SEGC+!SEGB+ SEGA) ,//Char = S,   offset=140
( SEGD+ SEGE+ SEGF+ SEGG) , (!SEGC+!SEGB+!SEGA) ,//Char = T,   offset=144
( SEGD+ SEGE+ SEGF+!SEGG) , ( SEGC+ SEGB+!SEGA) ,//Char = U,   offset=148
(!SEGD+!SEGE+!SEGF+!SEGG) , (!SEGC+!SEGB+!SEGA) ,//Char = V,   offset=152
(!SEGD+ SEGE+ SEGF+!SEGG) , ( SEGC+ SEGB+!SEGA) ,//Char = W,   offset=156
(!SEGD+!SEGE+!SEGF+!SEGG) , (!SEGC+!SEGB+!SEGA) ,//Char = X,   offset=160
(!SEGD+!SEGE+!SEGF+!SEGG) , (!SEGC+!SEGB+!SEGA) ,//Char = Y,   offset=164
( SEGD+!SEGE+!SEGF+!SEGG) , (!SEGC+!SEGB+ SEGA) ,//Char = Z,   offset=168
( SEGD+!SEGE+!SEGF+!SEGG) , (!SEGC+!SEGB+ SEGA) ,//Char = [,   offset=172
( SEGD+!SEGE+!SEGF+!SEGG) , (!SEGC+!SEGB+ SEGA) ,//Char = \,   offset=176
( SEGD+!SEGE+!SEGF+!SEGG) , (!SEGC+!SEGB+ SEGA) ,//Char = ],   offset=180
( SEGD+!SEGE+!SEGF+!SEGG) , (!SEGC+!SEGB+ SEGA) ,//Char = ^,   offset=184
( SEGD+!SEGE+!SEGF+!SEGG) , (!SEGC+!SEGB+ SEGA) ,//Char = _,   offset=188
( SEGD+!SEGE+!SEGF+!SEGG) , (!SEGC+!SEGB+ SEGA) ,//Char = `,   offset=192

};

unsigned char bLCD_CharPosition;


void SLCD_Init(void)
{
	SIM->SCGC5 |= SIM_SCGC5_SLCD_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

    // configure pins for LCD operation
    PORTC->PCR[20] = 0x00000000;     //VLL2
    PORTC->PCR[21] = 0x00000000;     //VLL1
    PORTC->PCR[22] = 0x00000000;     //VCAP2
    PORTC->PCR[23] = 0x00000000;     //VCAP1



    // Enable IRCLK
    MCG->C1 = MCG_C1_IRCLKEN_MASK | MCG_C1_IREFSTEN_MASK;
    //0 32KHZ internal reference clock; 1= 4MHz irc
    MCG->C2 &= ~MCG_C2_IRCS_MASK;

    //vfnLCD_interrupt_init();


    LCD->GCR = 0x0;
    LCD->AR  = 0x0;

    /* LCD configurartion according to */
    LCD->GCR = (   LCD_GCR_RVEN_MASK*_LCDRVEN
                 | LCD_GCR_RVTRIM(_LCDRVTRIM)       //0-15
                 | LCD_GCR_CPSEL_MASK*_LCDCPSEL
                 | LCD_GCR_LADJ(_LCDLOADADJUST)     //0-3*/
                 | LCD_GCR_VSUPPLY_MASK*_LCDSUPPLY  //0-1*/
                 |!LCD_GCR_FDCIEN_MASK
                 | LCD_GCR_ALTDIV(_LCDALTDIV)       //0-3
                 |!LCD_GCR_LCDDOZE_MASK
                 |!LCD_GCR_LCDSTP_MASK
                 |!LCD_GCR_LCDEN_MASK               //WILL BE ENABLE ON SUBSEQUENT STEP
                 | LCD_GCR_SOURCE_MASK*_LCDCLKSOURCE
                 | LCD_GCR_ALTSOURCE_MASK*_LCDALRCLKSOURCE
                 | LCD_GCR_LCLK(_LCDLCK)            //0-7
                 | LCD_GCR_DUTY(_LCDDUTY)           //0-7
              );


    // Enable LCD pins and Configure BackPlanes
    SLCD_EnablePins();

    LCD->GCR |= LCD_GCR_LCDEN_MASK;

    /* Configure LCD Auxiliar Register*/
    LCD->AR = LCD_AR_BRATE(_LCDBLINKRATE); // all other flags set as zero
}


void SLCD_EnablePins(void)
{
    unsigned char 		i;
   	unsigned long int *p_pen;
   	unsigned char 		pen_offset;   // 0 or 1   
   	unsigned char 		pen_bit;      // 0 to 31

   	LCD->PEN[0]	 = 0x0;
   	LCD->PEN[1]  = 0x0;
   	LCD->BPEN[0] = 0x0;
   	LCD->BPEN[1] = 0x0;
   
   	p_pen = (unsigned long int *)&LCD->PEN[0];

    for (i=0;i<_LCDUSEDPINS;i++) 
    {
      	pen_offset = WF_ORDERING_TABLE[i]/32;
      	pen_bit    = WF_ORDERING_TABLE[i]%32;
      	p_pen[pen_offset] |= MASK_BIT[pen_bit];
      	
      	// Pin is a backplane
      	if (i>= _LCDFRONTPLANES)    
      	{
      	    // Enable  BPEN 
            p_pen[pen_offset+2] |= MASK_BIT[pen_bit];  
            // fill with 0x01, 0x02, etc 
            LCD->WF8B[(unsigned char)WF_ORDERING_TABLE[i]] = MASK_BIT[i - _LCDFRONTPLANES];
      	} 
    }
}


void SLCD_WriteMsg(unsigned char *lbpMessage)
{
    unsigned char lbSize = 0;          
    bLCD_CharPosition = 0;  //Home display
    while (lbSize<_CHARNUM && *lbpMessage) 
    {
    	SLCD_WriteChar (*lbpMessage++);
      	lbSize++;     
    }
    
    if (lbSize<_CHARNUM) {
    	while (lbSize++< _CHARNUM) SLCD_WriteChar (BLANK_CHARACTER);  // complete data with blanks
	}
		
}


void SLCD_WriteChar(unsigned char lbValue)
{                                                                                                                         
    unsigned char char_val;                                                                                                 
    unsigned char temp;                                                                                                     
                                                                                                                    
    unsigned char *lbpLCDWF;                                                                                                
    unsigned char lbCounter;                                                                                                
                                                                                                                    
    unsigned short int arrayOffset;                                                                                             
    unsigned char position;                                                                                                 
                                                                                                                    
                                                                                                                    
  //lbpLCDWF = (unsigned char *)&LCD_WF3TO0;
    lbpLCDWF = (unsigned char *)&LCD->WF[0];
                                                                                                                          
                                                                                                                          
    /*only ascci character if value not writeable write as @*/                                                                
                                                                                                                          
    if (lbValue>='a' && lbValue<='z') lbValue -= 32; // UpperCase                                                   
    if (lbValue<ASCCI_TABLE_START || lbValue >ASCCI_TABLE_END) lbValue = BLANK_CHARACTER;  // default value as space
                                                                                                                    
    lbValue -=ASCCI_TABLE_START;        // Remove the offset to search in the ascci table                           
                                                                                                                    
    arrayOffset = (lbValue * _CHAR_SIZE); // Compensate matrix offset                                               
                                                                                                                          
    // ensure bLCD position is in valid limit                                                                                 
    lbCounter =0;  //number of writings to complete one char                                                        
    while (lbCounter<_CHAR_SIZE  && bLCD_CharPosition < _CHARNUM )                                                  
    {                                                                                                               
        position = (bLCD_CharPosition) *_LCDTYPE + lbCounter;                                                 
        temp=0;                                                                                               
        if (lbCounter==1)                                                                                     
        {                                                                                                     
        	temp = lbpLCDWF[WF_ORDERING_TABLE[position]] & 0x01;//bit 0 has the special symbol information    
        }                                                                                                     
                                                                                                              
        char_val = ASCII_TO_WF_CODIFICATION_TABLE[arrayOffset + lbCounter];                                   
                                                                                                              
        lbpLCDWF[WF_ORDERING_TABLE[position]] = char_val | temp;                                              
                                                                                                              
        //  if (char_val==0) lbCounter = _CHAR_SIZE; //end of this character                                  
        lbCounter++;                                                                                          
    }                                                                                                               
    bLCD_CharPosition++;                                                                                            
                                                                                                                    
}                                                                                                                         