#include "MKL46Z4.h"
#include "slcd.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#define PIN(x)                 (1 << x)

/* Macros for  Green LED*/
#define GREEN_LED             (5)
#define GREEN_LED_ON()       PTD->PCOR |= PIN(GREEN_LED) ;
#define GREEN_LED_OFF()      PTD->PSOR |= PIN(GREEN_LED) ;
#define GREEN_LED_TOGGLE()   PTD->PTOR |= PIN(GREEN_LED) ;

/* Macros for  Red LED*/
#define RED_LED             (29)
#define RED_LED_ON()       PTE->PCOR |= PIN(RED_LED) ;
#define RED_LED_OFF()      PTE->PSOR |= PIN(RED_LED) ; 
#define RED_LED_TOGGLE()   PTE->PTOR |= PIN(RED_LED) ; 

/* Define SWitch 1(SW1) and Switch 2(SW3) */
#define SW1             (3)
// SW2 is connected to PTC12
#define SW2             (12)
// Setting PE = 1 and PS = 1 for the corresponding pin to which the switches are connected.
#define ENABLE_PULLUP_RESISTOR  (1 << 1 | 1 << 0)


/* Macros for I2C Bus*/
#define I2C0_SCL                           (24)
#define I2C0_SDA                           (25)
#define EXTEND_READ_BIT(x)                 ((x<<1)|(0x01))
#define EXTEND_WRITE_BIT(x)                (x<<1) 

/* Macros for Magnetometer Device*/
#define MAG_DEVICE_ADDRESS                 0x0E
#define MAG_CTRL_REG1                      0x10
#define MAG_CTRL_REG2                      0x11
#define MAG_DR_STATUS                      0x00
#define MAG_OUT_X_MSB                      0x01
#define MAG_OUT_X_LSB                      0x02
#define MAG_OUT_Y_MSB                      0x03
#define MAG_OUT_Y_LSB                      0x04
#define MAG_OUT_Z_MSB                      0x05
#define MAG_OUT_Z_LSB                      0x06
#define MAG_WHO_AM_I_ADDRESS               0x07
#define MAG_SYSMOD                         0x08
#define MAG_OFF_X_MSB                      0x09
#define MAG_OFF_X_LSB                      0x0A
#define MAG_OFF_Y_MSB                      0x0B
#define MAG_OFF_Y_LSB                      0x0C
#define MAG_OFF_Z_MSB                      0x0D
#define MAG_OFF_Z_LSB                      0x0E
#define MAG_WHO_AM_I_ID                    0xC4


/* Macros for SysticTimer*/
#define BUSCLOCK_HZ (13.98 * 1000000)
#define BUSCLOCK_PERIOD_SEC ( 1 / BUSCLOCK_HZ )

/* Macros for math utils*/
#define RAD_TO_DEG_RATIO 57.29

/* Define States*/
typedef enum {
	STOP,
	RUN,
	ACQ,
	CAL,
} enumECompassState;

enumECompassState eCompassState = STOP;

bool is_SW3_pressed = false;
bool is_SW1_pressed = false;
bool firstTime = true;
unsigned char    sLCD_Msg[5] = "";

/* Defining variables to be used for acquiring and processing Magnetometer Data*/
unsigned char data_read[6];
short int DATA_READ_XYZ[3];
short int DATA_MAX_XYZ[3];
short int DATA_MIN_XYZ[3];
short int DATA_AVG_XYZ[3];
short int DATA_CAL_XYZ[3];
short int ANGLE;


/* Delay Function to allow processes to complete on I2C Bus*/
void delay()
{
	for(int i=1; i<1500; i++){}
}

/* Function to initialise LEDs*/
void LED_Init(void)
{
// GREEN_LED
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	PORTD->PCR[GREEN_LED] = PORT_PCR_MUX(1) ;
	PTD->PSOR |= PIN(GREEN_LED) ;
	PTD->PDDR |= PIN(GREEN_LED);
// RED_LED
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[RED_LED] = PORT_PCR_MUX(1) ;
	PTE->PSOR |= PIN(RED_LED) ;
	PTE->PDDR |= PIN(RED_LED);

}

/* Function to initialise Switch*/
void SWITCH_Init(void)
{
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	//Set configuration for Switch 1 (Input Mode)*/
	//PTC3 pin set to GPIO mode, PE(bit0) and PS(bit1) and  detect interrupt on falling edge 
	PORTC->PCR[SW1] |= PORT_PCR_MUX(1) | ENABLE_PULLUP_RESISTOR | PORT_PCR_IRQC(0xA);

	//Set configuration for Switch 2 (Input Mode)
	//PTC3 pin set to GPIO mode, PE(bit0) and PS(bit1) and  detect interrupt on falling edge 
	PORTC->PCR[SW2] |= PORT_PCR_MUX(1) | ENABLE_PULLUP_RESISTOR | PORT_PCR_IRQC(0xA);
	/*Set the pins direction to input (Set the corresponding bit to 0) */
	PTC->PDDR &= (~PIN(SW1))&(~PIN(SW2));

}


/* Initialise Timer*/
int32_t volatile msTicks = 0;

void SysTick_interrupt_Init()
{
	SysTick->LOAD = SystemCoreClock / 1000; //configured the SysTick to count in 1ms // 20971520U
	/* Select Core Clock & Enable SysTick & Enable Interrupt */
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void Delay_Systick (uint32_t TICK) 
{ 
	while (msTicks < TICK-1); 
	msTicks = 0; // Reset counter 
}

void SysTick_Handler (void) 
{ 
	msTicks++; // Increment counter 
}

#define GREEN_LED_HZ 1
#define RED_LED_HZ 2
#define GREEN_LED_PERIOD_MS 1000/GREEN_LED_HZ
#define RED_LED_PERIOD_MS 1000/RED_LED_HZ

void Blink_Green_LED()
{
	GREEN_LED_TOGGLE();
	Delay_Systick(GREEN_LED_PERIOD_MS);

}

void Blink_Red_LED()
{
	RED_LED_TOGGLE();
	Delay_Systick(RED_LED_PERIOD_MS);

}


/* Function to initialise I2C0*/
void I2C0_Init(void)
{
	// Enable I2C by providing Gate Clock to I2C0
	SIM->SCGC4|=SIM_SCGC4_I2C0_MASK;
	// Setting Pins corresponding of I2C0 clock and Data to GPIO pins
		PORTE->PCR[I2C0_SCL]|= PORT_PCR_MUX(1) | PORT_PCR_MUX(5);
		PORTE->PCR[I2C0_SDA]|= PORT_PCR_MUX(1) | PORT_PCR_MUX(5);
	// Enabling pullup resistor I2C0 Clock
		PORTE->PCR[I2C0_SCL]|= PORT_PCR_PS_MASK |PORT_PCR_PE_MASK;
	// Enabling pullup resistor I2C0 Data
		PORTE->PCR[I2C0_SDA]|= PORT_PCR_PS_MASK |PORT_PCR_PE_MASK;
	// Set the direction of the I2C SCL register to output
		PTE->PDDR |= PIN(I2C0_SCL);
	// Set the direction of the I2C SDA register to input
		PTE->PDDR &= ~PIN(I2C0_SDA);
	//I2C baud rate = bus speed (Hz)/(mul × SCL divider)
	//SDA hold time = bus period (s) × mul × SDA hold value
	//SCL start hold time = bus period (s) × mul × SCL start hold value
	//SCL stop hold time = bus period (s) × mul × SCL stop hold value
	//ICR	SCL divider  SDA hold  SCL hold (start)  SCL hold(stop) :00 20 7 6 11 4
	I2C0->F|=I2C_F_MULT(02);
	I2C0->F|=I2C_F_ICR(0x00);
	// Enable the Module operation
	I2C0->C1|=I2C_C1_IICEN_MASK;
	
}


void PORTC_PORTD_IRQHandler(void)
{
// --------------------------------------------------------------------
	int SW1_Status, SW2_Status; 
	SW1_Status=(PORTC->PCR[SW1] & PIN(24))>>24;
	SW2_Status=(PORTC->PCR[SW2] & PIN(24))>>24;

	if (SW1_Status==1)
	{
		is_SW1_pressed=1;      
		PORTC->PCR[SW1] |= PORT_PCR_ISF_MASK; //Clear Interrupt Status Flag
	}
	if (SW2_Status==1)
	{
		is_SW3_pressed=1;  
		PORTC->PCR[SW2] |= PORT_PCR_ISF_MASK; //Clear Interrupt Status Flag
	}

}

#define I2C_Write(data) I2C0->D = data

#define I2C_Read() I2C0->D

void I2C_Wait() 
{
	while((I2C0->S & I2C_S_IICIF_MASK) == 0){}
	// Clear flag bit 
	I2C0->S |= I2C_S_IICIF_MASK;
}

void I2C_Wait_ACK() 
{
	// wait for flag to be 0
	while ((I2C0->S & I2C_S_RXAK_MASK) != 0){}
}

void I2C_Start()
{
	//Set I2C in Transmit mode
	I2C0->C1 |=I2C_C1_TX_MASK;
	// Send Start bit
	I2C0->C1 |=I2C_C1_MST_MASK;
}

void I2C_Stop()
{
	//Send Stop Bit
	I2C0->C1 &= (~I2C_C1_MST_MASK);
	// Clear Transmit Nack by setting TXAK to 0
	I2C0->C1 &= ~(I2C_C1_TXAK_MASK);
}

void I2C_Repeated_start()
{
	I2C0->C1 |= I2C_C1_RSTA_MASK;
}

void I2C_NAck()
{
	I2C0->C1 |= I2C_C1_TXAK_MASK;
}

void I2C_Set_RX_mode()
{
	I2C0->C1 &= (~I2C_C1_TX_MASK);
}


unsigned char I2C_SingleByteRead(unsigned char device_address, unsigned char reg_address)
{   
	unsigned char DATA =0;
	I2C_Start();
	I2C_Write(EXTEND_WRITE_BIT(device_address));
	I2C_Wait();
	I2C_Wait_ACK() ;

	I2C_Write(reg_address);
	I2C_Wait();
	I2C_Wait_ACK() ;

	I2C_Repeated_start();
	I2C_Write(EXTEND_READ_BIT(device_address));
	I2C_NAck();
	I2C_Wait();
	I2C_Wait_ACK() ;

	I2C_Set_RX_mode();
	// Dummy Data
	DATA = I2C_Read();
	I2C_Wait();
	I2C_Stop();
	
	//Read Magnetometer Data
	DATA = I2C_Read();
	delay();
	return DATA;
}

void I2C_MultipleByteRead(unsigned char device_address,unsigned char reg_address, int max_count)
{   
	unsigned char DATA_DUMMY = 0;
	I2C_Start();
	I2C_Write(EXTEND_WRITE_BIT(device_address));
	I2C_Wait();
	I2C_Wait_ACK();

	I2C_Write(reg_address);
	I2C_Wait();
	I2C_Wait_ACK();

	I2C_Repeated_start();
	I2C_Write(EXTEND_READ_BIT(device_address));
	I2C_Wait();
	I2C_Wait_ACK();

	I2C_Set_RX_mode();
	DATA_DUMMY = I2C_Read();
	for(int i = 0; i < max_count; i++)
	{
		if(i<(max_count-2))
		{
			I2C_Wait();
			data_read[i]=I2C_Read();
		}
		else
		{
			I2C_NAck();
			I2C_Wait();
			data_read[i]=I2C_Read();
			i = i+1;
			I2C_Wait();
			I2C_Stop();
			data_read[i]=I2C_Read();

		}
	}
	delay();	
}

void I2C_SingleByteWrite(unsigned char device_address, unsigned char reg_address, unsigned char DATA)
{
	I2C_Start();
	I2C_Write(EXTEND_WRITE_BIT(device_address));
	I2C_Wait();
	I2C_Wait_ACK();

	I2C_Write(reg_address);
	I2C_Wait();
	I2C_Wait_ACK();

	I2C_Write(DATA);
	I2C_Wait();
	I2C_Wait_ACK();

	I2C_Stop();
	delay();
}


void I2C_MultipleByteWrite(unsigned char device_address, unsigned char reg_address, int max_count, unsigned char data_wr[])
{
	I2C_Start();
	I2C_Write(EXTEND_WRITE_BIT(device_address));
	I2C_Wait();
	I2C_Wait_ACK();

	I2C_Write(reg_address);
	I2C_Wait();
	I2C_Wait_ACK();

	for(int i = 0;i < max_count;i++)
	{
		I2C_Write(data_wr[i]);
		I2C_Wait();
		I2C_Wait_ACK() ;
	}
	I2C_Stop();

	delay();
}


/* Magnetometer*/
void Magnetometer_Init(void)
{	unsigned char MAG_DEVICE_ID;
	MAG_DEVICE_ID = I2C_SingleByteRead(MAG_DEVICE_ADDRESS, MAG_WHO_AM_I_ADDRESS);
	// checking the device ID (0xC4) of magnetometer
	while(MAG_DEVICE_ID != MAG_WHO_AM_I_ID)
	{
		SLCD_WriteMsg((unsigned char *)"Err");
	}

	I2C_SingleByteWrite(MAG_DEVICE_ADDRESS,MAG_CTRL_REG1,0x00);
	I2C_SingleByteWrite(MAG_DEVICE_ADDRESS,MAG_CTRL_REG2,0x80);
	I2C_SingleByteWrite(MAG_DEVICE_ADDRESS,MAG_CTRL_REG1,0x01);

}

void Magnetometer_Acq(void)
{
    const int axis_count = 3;
    const int data_length = 6;

    I2C_MultipleByteRead(MAG_DEVICE_ADDRESS, MAG_OUT_X_MSB, data_length);

    for (int i = 0; i < axis_count; i++){
        DATA_READ_XYZ[i] = (short int)((data_read[2 * i] << 8) | data_read[2 * i + 1]);
    }
    
    for (int i = 0; i < axis_count; i++){
        if ((DATA_MAX_XYZ[i] == 0) && (DATA_MIN_XYZ[i] == 0)){
            DATA_MAX_XYZ[i] = DATA_READ_XYZ[i];
            DATA_MIN_XYZ[i] = DATA_READ_XYZ[i];
        }
        else{
			// Finding the Maximum value along each axis
            if (DATA_READ_XYZ[i] > DATA_MAX_XYZ[i]){
                DATA_MAX_XYZ[i] = DATA_READ_XYZ[i];
            }
            
            // Finding the Minimum value along each axis
            if (DATA_READ_XYZ[i] < DATA_MIN_XYZ[i]){
                DATA_MIN_XYZ[i] = DATA_READ_XYZ[i];
            }
        }
    }
}


void Magnetometer_Cal(void)
{
	for(int i=0;i<3;i++){   
		DATA_AVG_XYZ[i]=(DATA_MAX_XYZ[i]+DATA_MIN_XYZ[i])/2;
	}
}


void Magnetometer_Run(void)
{
    const int axis_count = 3;
    const int data_length = 6;
    
    I2C_MultipleByteRead(MAG_DEVICE_ADDRESS, MAG_OUT_X_MSB, data_length);
    
    for (int i = 0; i < axis_count; i++){
        DATA_READ_XYZ[i] = (short int)((data_read[2 * i] << 8) | data_read[2 * i + 1]);
    }
    
    for (int i = 0; i < axis_count; i++){
        DATA_CAL_XYZ[i] = DATA_READ_XYZ[i] - DATA_AVG_XYZ[i];
    }

    if (DATA_CAL_XYZ[1] == 0){
        if (DATA_CAL_XYZ[0] > 0){
            ANGLE = 0;
        }
        else if (DATA_CAL_XYZ[0] < 0){
            ANGLE = 180;
        }
    }
    else if (DATA_CAL_XYZ[1] < 0){
        ANGLE = 270 - (atan((double)DATA_CAL_XYZ[0] / (double)DATA_CAL_XYZ[1]) * RAD_TO_DEG_RATIO);
    }
    else{
        ANGLE = 90 - (atan((double)DATA_CAL_XYZ[0] / (double)DATA_CAL_XYZ[1]) * RAD_TO_DEG_RATIO);
    }
}

int main(void)
{

	SLCD_Init();
	SysTick_interrupt_Init();
	LED_Init();
	SWITCH_Init();
	I2C0_Init();
	Magnetometer_Init();
	NVIC_EnableIRQ(PORTC_PORTD_IRQn);

    while(1){
        if(is_SW1_pressed == true){
          // Clear the flag
            is_SW1_pressed = false;
            if(eCompassState == STOP && firstTime == true){
				firstTime = false;
				RED_LED_OFF()
                eCompassState = ACQ;
            }else if(eCompassState == ACQ){
                eCompassState = CAL;
            }else if(eCompassState == CAL){
                eCompassState = RUN;
            }else if(eCompassState == RUN){
				GREEN_LED_OFF()
				eCompassState = STOP;
			}else if(eCompassState == STOP){
				RED_LED_OFF()
				eCompassState = RUN;
			}
        }else if(is_SW3_pressed == true){
          // Clear the flag
            is_SW3_pressed = false;
			firstTime = true;
			GREEN_LED_OFF()
			eCompassState = STOP;
            
        }
        if(eCompassState == STOP){
			Blink_Red_LED();
			//Display STOP Message
			SLCD_WriteMsg((unsigned char *)"ST0P");
        }else if(eCompassState == RUN){
			Magnetometer_Run();
			Blink_Green_LED();
			snprintf(sLCD_Msg,5,"%4d",ANGLE);
			SLCD_WriteMsg(sLCD_Msg);
        }else if(eCompassState == ACQ){
			Magnetometer_Acq();
			Blink_Green_LED();
        	SLCD_WriteMsg((unsigned char *)"MACQ");
        }else if(eCompassState == CAL){
        	Magnetometer_Cal();
			Blink_Green_LED();
        	SLCD_WriteMsg((unsigned char *)"MCAL");

        }
    }
}
