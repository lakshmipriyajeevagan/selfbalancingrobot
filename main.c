#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include <string.h>
/***/
//from hello world sample to enable UART
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include <inc/hw_ints.h>
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/rom_map.h"

#define GPIO_PA0_U0RX       0x00000001            // UART PIN ADDRESS FOR UART RX
#define GPIO_PA1_U0TX       0x00000401            // UART PIN ADDRESS FOR UART TX
/***/

#define  SLAVE_ADDR 0x68     /* 0110 1000 */ //slave I2C address
 
void EnableInterrupts(void);
void init_uart(void);
double atanb(double x1, double y);
void I2C3_init(void);
void MPU6050_init(void);
char I2C3_byteWrite(int slaveAddr, char memAddr, char data);
char I2C3_read(int slaveAddr, char memAddr, int byteCount, char* data);
void PID_Controller(void);
void ftoa(float n, char *res, int afterpoint);
int intToStr(int x, char str[], int d);
void reverse(char *str, int len);
void timer0A_init(void);
void timer0_handler(void);
void delayMs(int n);
void delayUs(int n);

void motor_init(void);
void pwm_init(void);
void set_dutycycle1(int i);
void forward(void);
void backward(void);
void stop(void);
char I2C_wait_till_done(void);
void print_double(double input);

/***/
//adding a function for UART display
int uart (void);

double accel_angle = 0, new_accel_angle, angle_est, prev_angle, count = 0;
double set_point = 0, Total_error = 0, Deviation, correction, duty = 0;
float axf, ayf, azf, gxf, gyf, gzf, sum; // accel, gyro axes variables
int set_flag = 0, set_cnt = 0,flag = 0;
int var ;

char accel_data[14], buffer[20];
int16_t ax, ay, az, gx, gy, gz;  //accel, gyro axes variables
double yz;


//using UART for debugging (from hello world sample)
void ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//UART functions
int uart (void)
{
		FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN); 

    //
    // Initialize the UART.
    //
    ConfigureUART();
} 

int main(void)
{		
		uart();
	
		/*********debug UART*****************

		int i = 0;
		while (i<10){
				while ((UART0_FR_R & 0x20) != 0){
						GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
				}
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0); 
				while ((UART0_FR_R & 0x20) != 0){;}
		//			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
			//GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
				UART0_DR_R = 'w';

				i++;
		}	
		GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
		GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, 0X0E); //new line added

		while (1){
				while ((UART0_FR_R & 0x20) != 0){;
						GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
				}
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
				UART0_DR_R = 'r';
		}

		**************************************/


		
		pwm_init();
		I2C3_init();
		timer0A_init();
		init_uart();
		MPU6050_init();

		while (1)
		{
				// uart();  abcd
				//UARTprintf("%d\n",__LINE__);

				I2C3_read(SLAVE_ADDR, 0x3B, 14, accel_data);

				ax = (int16_t) ((accel_data[0] << 8) | (accel_data[1]));
				ay = (int16_t) ((accel_data[2] << 8) | (accel_data[3]));
				az = (int16_t) ((accel_data[4] << 8) | (accel_data[5]));

				gx = (int16_t) ((accel_data[8] << 8) | (accel_data[9]));
				gy = (int16_t) ((accel_data[10] << 8) | (accel_data[11]));
				gz = (int16_t) (accel_data[12] << 8) | (accel_data[13]);

				// Calibration of the sensor by adding / subtracting the offsets

				axf = (((float) ax) / 16384.0) - 0.04;  //(2^16bits)/(2+2) in terms of g
				ayf = (((float) ay) / 16384.0) + 0.014;
				azf = (((float) az) / 16384.0) + 0.07;

				gxf = ((float) gx / 131.0) + 0.53; // (2^16bits)/(250+250) in dps
				gyf = ((float) gy / 131.0);
				gzf = ((float) gz / 131.0) + 1.05;

				PID_Controller();
				//UARTprintf("%d\n",__LINE__);
				//to display estimated angle and correction factor on UART
				//	uart ();  


				//UARTprintf("%d\n",__LINE__);
				print_double(correction);
		}

}

/* initialize I2C3 as master and the port pins */
void I2C3_init(void)
{
		//uart(); abcd
		//UARTprintf("\nI2C3 called\t");
		//enale clock for I2C and GPIO registers
		SYSCTL_RCGCI2C_R |= 0x08; /* enable clock to I2C3 */
		SYSCTL_RCGCGPIO_R |= 0x08; /* enable clock to GPIOD */

		/* PORTD 1 (SDA), 0 (SCL) for I2C3 */
		//AFSEL (Alternate Function Select) register to use port D
		GPIO_PORTD_AFSEL_R |= 0x03; /* PORTD 1, 0 for I2C3  */ //D0=scl D1=sda
		
		//Port Control register to be assigned a bit field encoding value  
		GPIO_PORTD_PCTL_R &= ~0x000000FF; /* PORTD 1, 0 for I2C3 */
		GPIO_PORTD_PCTL_R |= 0x00000033;
		
		//Digital enable
		GPIO_PORTD_DEN_R |= 0x03; /* PORTD 1, 0 as digital pins */
		
		//make D1 open drain (refer datasheet)
		GPIO_PORTD_ODR_R |= 0x02; /* PORTD 1 as open drain */

		I2C3_MCR_R = 0x10; /* master mode */
		I2C3_MTPR_R = 7; /* 100 kHz @ 16 MHz */
}

void init_uart(void)  //for debugging
{   //use masking for all the initializations if doesn't work
		// uart(); abcd
	 // UARTprintf("\n init UART called\t");

    SYSCTL_RCGC2_R |= 0x01;          //Clock and Enable Port A wrt functionalities
    SYSCTL_RCGCUART_R |= 0x01;       //Enable and provide Clock to UART0
    SYSCTL_RCGCGPIO_R |= 0x01;       //Clock to PORTA
    UART0_CTL_R |= 0x00;             //Disable UART for setting purpose
    UART0_IBRD_R |= 0x08;            //For 115200bps 8 = int [ system_clk i.e. 16MHz / (16 * baud_rate)]
    UART0_FBRD_R |= 0x2C;            //For 115200bps 44 = int [(fractional part * 64) + 0.5]
    UART0_LCRH_R = 0x60;             //for 8-bit data size, no FIFO, 1 stop bit, no interrupt, no parity
    UART0_CTL_R = 0x301;             //for en, rxe, txe
    GPIO_PORTA_DEN_R |= 0x03;        //PA0,PA1 digital IO
    GPIO_PORTA_AFSEL_R |= 0x03;      //PA0,PA1 alternate functions rxe, txe
    GPIO_PORTA_PCTL_R |= 0x11;       //PA0 and PA1 pins for UART function
    GPIO_PORTA_AMSEL_R |= 0x00;      //To disable ADC (disabling analog functionality)

}

//stop reading val from mpu6050 every 20ms & then send pwm to motor driver
void timer0A_init(void)
{
		//uart();  abcd
	 // UARTprintf("\ntimer0A called\t");
    SYSCTL_RCGCTIMER_R |= 1;
    TIMER0_CTL_R = 0;  // DISABLE TIMER
    TIMER0_CFG_R = 0x04;  //16 BIT MODE
    TIMER0_TAMR_R = 0x02;  // DOWN COUNT AND PERIODIC
    TIMER0_TAILR_R = 20000 - 1;  //LOAD VALUE
    TIMER0_TAPR_R = 16 - 1;  //PRESCALER VALUE
    TIMER0_ICR_R = 0x1;  // CLEARING TIMEOUT INTERRUPTS
    TIMER0_IMR_R = 0X00000001;  //TIMEOUT ENABLE
    NVIC_PRI4_R = (NVIC_PRI4_R & 0x8FFFFFFF) | 0X30000000;
    NVIC_EN0_R = 0x00080000;
    TIMER0_CTL_R |= 0x01; //ENABLE TIMER AND START COUNTING
    EnableInterrupts();
		//UARTprintf("\nenable interrupts");
}

//timer interrupt overflow set at 20msec
void timer0_handler(void)
{
    int Kp = 2500, Ki = 5000, Kd = 100;  // Design 1 - Kp = 2500, Ki = 5000, Kd = 30
                                         // Design 2 - Kp = 2500, Ki = 1500, Kd = 100
    double present_error,dt = 0.02;   //alpha = 0.02; t=20msec which timer overflow interrupt
    TIMER0_ICR_R = 0x1;  //clear timeout

    yz = (double) sqrt(ayf * ayf + azf * azf);
		// uart();   abcd
		//UARTprintf("yz = %d\t", yz);
    accel_angle = 0.8* accel_angle + 0.2 * atan2((double) (axf), yz) * 180 / 3.141592 ;
		//UARTprintf("accel angle: %d\t", accel_angle);
    prev_angle = angle_est;
    angle_est = ((0.99) * (angle_est - (double) (gyf * dt)) + (0.01 * accel_angle));   //Complementary filter

    Deviation = (prev_angle - set_point) - (angle_est - set_point);
    present_error = set_point - angle_est;

    //PID algorithm
   // correction = Kp * (present_error) + (Ki * Total_error) + Kd * Deviation/dt;   //Deviation*50 = Deviation/dt
		correction = Kp * (present_error) + (Ki * Total_error) + (Kd * (Deviation/dt));  
		Total_error = Total_error + present_error*dt;
    //UARTprintf("%d\n", correction);
}

void PID_Controller(void)
{
		//uart(); abcd
	  //UARTprintf("\tPID controller called\t");

		timer0_handler();
     if(angle_est>0){
				if(correction < 0){
					duty = correction;
					set_dutycycle1((-1)* (int)(duty));  //casting double to int
					print_double(duty);
					backward();
		    }
     }

     else{
			  if(correction > 0){
					duty =  correction;
					set_dutycycle1((int)duty);
					forward();
			  }
      }
}

void pwm_init(void)
{
		//uart();  abcd
	  //UARTprintf("\n pwm_init called\t");

    SYSCTL_RCGC2_R |= 0x00000031;            // Enable clock to the necessary ports
    GPIO_PORTE_DIR_R |= 0x0E;                // Specify the port E pins as output
    GPIO_PORTE_DEN_R |= 0x0E;                // Provide a digital enable to the portE GPIO pins
    GPIO_PORTA_DIR_R |= 0x80;                // Specify the port A pins as output
    GPIO_PORTA_DEN_R |= 0x80;                // Provide digital enable to the portA GPIO pins

    SYSCTL_RCGCPWM_R |= 2;                   // Enable clock to the PWM module
    SYSCTL_RCC_R &= ~0x00100000;             // Use the system clock directly without any frequency division
    GPIO_PORTF_LOCK_R = 0x4C4F434B;          // unlock GPIO Port F
    GPIO_PORTF_CR_R |= 0x03;                 // Enable Port F0 and F1 pins at the commit register
    GPIO_PORTF_DIR_R |= 0x03;                // Specify direction as output for portF pins
    GPIO_PORTF_DEN_R |= 0x03;                // Digital enable to PF0 and PF1
    GPIO_PORTF_AFSEL_R |= 0x01;              // Use of Alternate function select as using PWM peripheral instead of GPIO
    GPIO_PORTF_PCTL_R &= ~0x0000000F;        // Clearing Port control register
    GPIO_PORTF_PCTL_R |= 0x00000005;         // Assign Bit Field Encoding value to the Port Control Register

    GPIO_PORTA_AFSEL_R |= 0x80;
    GPIO_PORTA_PCTL_R &= ~0xF0000000;
    GPIO_PORTA_PCTL_R |= 0x50000000;

    PWM1_1_CTL_R = 0;                        // stop counter before initialization of certain variables
    PWM1_2_CTL_R = 0;

    PWM1_1_GENB_R = 0x0000008C;              
    PWM1_2_GENA_R = 0x0000008C;
    PWM1_1_LOAD_R = 16000;                   // Assign values to the LOAD register in the PWM generators
    PWM1_2_LOAD_R = 16000;
    PWM1_1_CTL_R = 1;                        // enable counter after initialization
    PWM1_2_CTL_R = 1;

}
void set_dutycycle1(int i)
{
    i = 13000-i;                            // Provide an offset when the bot at upright position i.e. angle zero
    PWM1_1_CMPA_R = i;                      // Assign value to the compare register
    PWM1_2_CMPA_R = i;
}

void forward(void)
{
		while ((UART0_FR_R & 0x20) != 0);
		UART0_DR_R = 'f';
    PWM1_ENABLE_R = 0x18;
    GPIO_PORTE_DATA_R |= 0x4;
    GPIO_PORTE_DATA_R &= ~0x0A;
    GPIO_PORTF_DATA_R |= 0x2;
}
void backward(void)
{
		while ((UART0_FR_R & 0x20) != 0);
		UART0_DR_R = 'b';
    PWM1_ENABLE_R = 0x18;
    GPIO_PORTE_DATA_R |= 0xA;
    GPIO_PORTE_DATA_R &= ~0x04;
    GPIO_PORTF_DATA_R &= ~0x2;

}
void stop(void)
{
    PWM1_ENABLE_R = 0x00;
    GPIO_PORTE_DATA_R &= ~0x0E;
    GPIO_PORTF_DATA_R &= ~0x02;

}


void MPU6050_init(void)  // Why writing those values MDR_R ?  ### Link: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
{
		//uart();  abcd
	 // UARTprintf("\nmpu6050 called\t");
	  
    I2C3_byteWrite(SLAVE_ADDR, 0x6B, 0x01); // clock 8 mhz pll with x axis gyro reference
		//UARTprintf("\t%d\n", __LINE__);
  	//UARTprintf("\tbefore delay\t");
  	delayMs(100);
		//UARTprintf("\tafter delay\t");
	
    I2C3_byteWrite(SLAVE_ADDR, 0x68, 0x06); // signal path reset (resets the sensor path of accelerometer and gyroscope)
 // UARTprintf("\t%d\n", __LINE__);  
	delayMs(100);
    
	I2C3_byteWrite(SLAVE_ADDR, 0x6A, 0x00); // i2c_if_dis = 0  // resetting sensor registers of acc and gyro
 // UARTprintf("\t%d\n", __LINE__);  
	delayMs(100);
    
	I2C3_byteWrite(SLAVE_ADDR, 0x1A, 0x00); //fsync and dlpf disabled
  //UARTprintf("\t%d\n", __LINE__);  
	delayMs(100);
    
		I2C3_byteWrite(SLAVE_ADDR, 0x19, 0x07); //sample rate set to 1 khz   [(gyro_output_freq i.e.8khz)/(1 + sampler_div i.e. 7)]
  //UARTprintf("\t%d\n", __LINE__);  
	delayMs(100);
    
		I2C3_byteWrite(SLAVE_ADDR, 0x1B, 0x00); // +/- 250dps gyrometer  configuration
  //  UARTprintf("\t%d\n", __LINE__);
		delayMs(100);
    
		I2C3_byteWrite(SLAVE_ADDR, 0x1C, 0x00); // +/- 2g  accelerometer configuration
   // UARTprintf("\t%d\n", __LINE__);
		delayMs(100);
		//UARTprintf("mpu done");
}

char I2C3_byteWrite(int slaveAddr, char memAddr, char data)
{
		//uart(); abcd
		//UARTprintf("\nIn I2C3 bytewrite");
    char error;
    I2C3_MSA_R = slaveAddr << 1;
    I2C3_MDR_R = memAddr;
    I2C3_MCS_R = 3;
	
	 // UARTprintf("\nwaitTillDone begins");
    error = I2C_wait_till_done();
		//UARTprintf("\twaitTillDone ends");

    if (error)
        return error;
    I2C3_MDR_R = data;
    I2C3_MCS_R = 5; //Master generates Stop bit while in Rx or Tx mode
		
		//UARTprintf("\tmcs");
    error = I2C_wait_till_done(); 
    while (I2C3_MCS_R & 0x40)  //  ???? Why do we need the next three lines ????  //
        ;
    error = I2C3_MCS_R & 0xE;
    if (error)
        return error;  
    return 0;
		
		//UARTprintf("\tI2C3 ends");
}

/* Wait until I2C master is not busy and return error code */
/* If there is no error, return 0 */
char I2C_wait_till_done(void)
{  
		static int i = 0;
		//uart();  abcd
		//UARTprintf("\t%d\n", i++);
	
		while (I2C3_MCS_R & 1)  
		//	UARTprintf("\tx\n");
				 /* wait until I2C master is not busy */
		return I2C3_MCS_R & 0xE; /* return I2C error code */
}

char I2C3_read(int slaveAddr, char memAddr, int byteCount, char* data)
{
    char error;

    if (byteCount <= 0)
        return -1; /* no read was performed */

    /* send slave address and starting address */
    I2C3_MSA_R = slaveAddr << 1;
    I2C3_MDR_R = memAddr;
    I2C3_MCS_R = 3; /* S-(saddr+w)-ACK-maddr-ACK */
    error = I2C_wait_till_done();
    if (error)
        return error;

    /* to change bus from write to read, send restart with slave addr */
    I2C3_MSA_R = (slaveAddr << 1) + 1; /* restart: -R-(saddr+r)-ACK */

    if (byteCount == 1) /* if last byte, don't ack */
        I2C3_MCS_R = 7; /* -data-run-stop- */
    else
        /* else ack */
        I2C3_MCS_R = 0xB; /* -data-ACK- */
    error = I2C_wait_till_done();
    if (error)
        return error;

    *data++ = I2C3_MDR_R; /* store the data received */

    if (--byteCount == 0) /* if single byte read, done */
    {
        while (I2C3_MCS_R & 0x40)
            ; /* wait until bus is not busy */
        return 0; /* no error */
    }

    /* read the rest of the bytes */
    while (byteCount > 1)
    {
        I2C3_MCS_R = 9; /* -data-ACK-automatically by board */
        error = I2C_wait_till_done();
        if (error)
            return error;
        byteCount--;
        *data++ = I2C3_MDR_R; /* store data received */
    }

    I2C3_MCS_R = 5; /* -data-NACK-P */
    error = I2C_wait_till_done();
    *data = I2C3_MDR_R; /* store data received */
    while (I2C3_MCS_R & 0x40)
        ; /* wait until bus is not busy */

    return 0; /* no error */
}

//arc tangent function
double atanb(double x1, double y)
{
    if (y == 0 && x1 > 0)
    {
        return (3.141592 / 2);
    }
    else if (y == 0 && x1 < 0)
    {
        return (-3.141592 / 2);
    }
    else
    {
        double a;
        double x = x1 / y;
        a = x - (1 / 3) * (x * x * x) + (1 / 5) * (x * x * x * x * x)
                - (1 / 7) * (x * x * x * x * x * x * x);
        return a;
    }
}
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

void reverse(char *str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

void ftoa(float n, char *res, int afterpoint)
{
    int i = 0, flag = 0;
    if (n < 0)
    {
        flag = 1;
        res[0] = '-';
        n = -1 * n;
    }
    // Extract integer part
    int ipart = (int) n;

    // Extract floating part
    float fpart = n - (float) ipart;

    // convert integer part to string
    if (flag)
    {
        i = intToStr(ipart, res + 1, 8);
    }
    else
    {
        i = intToStr(ipart, res, 8);
    }

    // check for display option after point
    if (afterpoint != 0)
    {
        if (flag)
        {
            res[i + 1] = '.';  // add dot
        }
        else
        {
            res[i] = '.';  // add dot
        }

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        int j = 0;
        for (j = 0; j < afterpoint; j++)
        {
            fpart = fpart * 10;
        }

        if (flag)
        {
            intToStr((int) fpart, res + i + 2, afterpoint);
        }
        else
        {
            intToStr((int) fpart, res + i + 1, afterpoint);
        }

    }
}


/* delay n milliseconds (16 MHz CPU clock) */
void delayMs(int n)
{
    int i, j;

    for (i = 0; i < n; i++)
        for (j = 0; j < 3180; j++)  // How to get value of 3180?????
        {
        } /* do nothing for 1 ms */
}

/* delay n microseconds (16 MHz CPU clock) */
void delayUs(int n)
{
    int i, j;

    for (i = 0; i < n; i++)
        for (j = 0; j < 3; j++)
        {
        } /* do nothing for 1 us */
}

void EnableInterrupts(void)   
{
    __asm ("    CPSIE  I\n");
}

void WaitForInterrupt(void)
{
    __asm ("    WFI\n");
}

void print_double(double input){
	char buffer1[20];
	ftoa(correction, buffer1, 4);  //float to ASCII
	int i = 0;
	while (buffer1[i] != '\0')
	{
			while ((UART0_FR_R & 0x20) != 0)   //if UART busy
					;
			UART0_DR_R = buffer1[i];
			i++;
	}
	while ((UART0_FR_R & 0x20) != 0);
	UART0_DR_R = 'w';

}
