/*
 * Team Id :  8401
 * Author List: Shivika Singh
 * Filename : task5-main.c
 * Theme : Supply Bot
 * Functions:buzzer_pin_config (),motors_config(),servo_config(),striker_config(),Xbee_config(),White_line_sensor_config(),
 *           port_init(),buzzer_on_off(),striker_servo(),stop(),forward(),timer5_init(),ADC_Conversion(unsigned char Ch),
 *           velocity (unsigned char left_motor, unsigned char right_motor),adc_init(),devices_init(),main()
 * Global Variables:flag, Left_white_line, Center_white_line, Right_white_line,data
 */ 

#define F_CPU 16000000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
//Global Variables
unsigned char ADC_Conversion(unsigned char);
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
unsigned char data; 
unsigned char left_motor;
unsigned char right_motor;







//************************************************************************************************************************//
//________________________________________________________________________________________________________________________//
/*The following code is for configuring ports and pins for various devices.The connection on arduino board are as follows:
*Pin 0 , 1 (Tx,Rx) : Xbee module
*Pin 2 : Buzzer module
*Pin 3 : Servo 
*Pin 4,5,6 : Striker 
*Pin 8,9,10,11,12,13 : Left and Right motor connections with motor driver
*Analog pins A0, A1, A2 : White line sensor */
//************************************************************************************************************************//
//________________________________________________________________________________________________________________________//


/* Function name : buzzer_pin_config
*  Input : NONE
*  Output : NONE
*  Logic:  Configures a given PORTD pin 2 for buzzer
* Example call: buzzer_pin_config ();
*/
void buzzer_pin_config ()     
{
	DDRD = 0b00000100;    /*setting PORTD  pin 2 as output for buzzer */
	PORTD= 0b00000000;    /*Setting PORTD pin  2 logic low to turnoff buzzer initially */
}
 

/* Function name : motors_config
*  Input : NONE
*  Output : NONE
*  Logic:  Configures a given PORTB pin 8,9,10,11,12,13 for left and right motors
* Example call: motors_config();
*/
void motors_config()
{
	DDRB = 0b11111111;    /*setting PORTB  pin 8,9,1 & 11,12,13 as output for left and right motors */
	PORTB= 0b00100001;    /*Setting PORTB pin 9,10 & 11,12  logic low to turnoff motors initially
	                         and setting PORTB pin 8 & pin 13 for velocity control of left and right motors */
} 


/* Function name : servo_config
*  Input : NONE
*  Output : NONE
*  Logic: Configures a given PORTD pin 3 for servo
* Example call: servo_config();
*/
void servo_config()
{
	DDRD = 0b00001000;    /*setting PORTD  pin 3 as output for servo */
	PORTD= 0b00000000;    /*Setting PORTD pin  3 logic low to turnoff servo initially  */
}



/* Function name : striker_config
*  Input : NONE
*  Output : NONE
*  Logic: Configures a given PORTD pin 4,5,6 for striker
* Example call: striker_config();
*/
void striker_config()
{
	DDRD = 0b01110000;    /*setting PORTD  pin 4,5,6 as output for striker,where pin 4 & pin5 are connected
	                         to IN3 & IN4 of motor driver respectively */
	
	PORTD= 0b00010000;    /*Setting PORTD pin  4,5,6 logic low to turnoff striker initially and setting pin 4 for velocity control */
}



/* Function name : Xbee_config
*  Input : NONE
*  Output : NONE
*  Logic: Configures Xbee module
* Example call:Xbee_config();
_____________________________*/
//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
//___________________________*/
void Xbee_config()
{
UCSR0A = 0x00;
UCSR0C = 0x06;
UBRR0L = 0x47; //11059200 Hz
UBRR0H = 0x00; //set baud rate hi
UCSR0B = 0x98;	
}


/* Function name : White_line_sensor_config
*  Input : NONE
*  Output : NONE
*  Logic: Configures PORTC analog pins A0, A1, A2 for white line sensors
* Example call: White_line_sensor_config();
*/
void White_line_sensor_config()
{
	DDRC = 0b00001000;    /*setting PORTC  pin A0,A1,A2 as input for servo */
	PORTC= 0b00000000;    /*Setting PORTC pin  A0,A1,A2 logic low to turnoff servo initially  */
}


/* Function name : port_init
*  Input : NONE
*  Output : NONE
*  Logic: Initializes all ports configured with buzzer,motors,servo,striker,white line sensor
* Example call:port_init();
*/
void port_init()
{
	buzzer_pin_config();
	motors_config();
	servo_config();
	striker_config();
	Xbee_config();
	White_line_sensor_config();
}

//********************************************************************************************************************//
//____________________________________________________________________________________________________________________//
//The following section contains function related to bot operation like :
// #Signal_reading from incoming  Xbee signal
// #buzzer on and off at reaching required node
// #striker on for striking at the coin
// #servo rotation 90 degree and 0 degree rotation for getting the striker back to its original position
// #Stopping at capital  or at nodes after hitting all coins
// #forward motion
// #Left side motion
// #right side motion
//*******************************************************************************************************************//
//___________________________________________________________________________________________________________________//
 
 /* Function name : buzzer_on_off
 *  Input : NONE
 *  Output : NONE
 *  Logic: Beeps the buzzer twice 
 * Example call: buzzer_on_off();
 */
 void buzzer_on_off()
 {
	 // We need to beep the buzzer twice on reaching the required node
	PORTD = 0b00000100;   // buzzer on 
	_delay_ms(1000);      // Wait for some time
	PORTD = 0b00000000;    // buzzer off
	_delay_ms(1000);      // Wait for some time
	
	PORTD = 0b00000100;   // buzzer on
	_delay_ms(1000);      // Wait for some time
	PORTD = 0b00000000;    // buzzer off
	_delay_ms(1000);      // Wait for some time
 }
 
 
 
 /* Function name : striker_servo()
 *  Input : NONE
 *  Output : NONE
 *  Logic: striker first is turned on and strikes the coin then servo moves from 0 degree to 90 degrees
           then striker is on but stopped at half rotation due to being stopped by servo at 90 , this is
		   done to make striker's angle with the coin perfect for sufficient strike .Finally the servo
		   goes back to 0 position and striker is ready for next hit.
 * Example call:striker_servo();
 */
 void striker_servo()
 {
	 //Initially the servo is at zero and on receiving strike signal ,the striker hits the coin then
	 //the servo goes to 90 degrees and striker completes half revolution because the servo stops striker at
	 //half revolution to make striker's angle with the coin perfect for sufficient strike .Finally the servo
	 //goes back to 0 position and striker is ready for next hit.
	 PORTD = 0b01010000;  
	 _delay_ms(1500);
	 PORTD = 0b00000000;
     _delay_ms(500);
	 
	 TCNT1 = 0;		     //Set timer1 count zero */
	 ICR1 = 2499;		 //Set TOP count for timer1 in ICR1 register */

	 /* Set Fast PWM, TOP in ICR1, Clear OC1A on compare match, clk/64 */
	 TCCR1A = (1<<WGM11)|(1<<COM1A1);
	 TCCR1B = (1<<WGM12)|(1<<WGM13)|(1<<CS10)|(1<<CS11);
	 
	  OCR1A = 65;	/* Set servo shaft at -90� position */
	  _delay_ms(500);
	  
	 PORTD = 0b01010000;
	 _delay_ms(1000);
	 PORTD = 0b00000000;
	 _delay_ms(500); 
	  
	 OCR1A = 175;	/* Set servo shaft at 0� position */
	 _delay_ms(1500);
  }
  
 
 
  /* Function name : stop
  *  Input : NONE
  *  Output : NONE
  *  Logic: Stops the motors by giving low value to both the motors
  * Example call: stop();
  */
  void stop()
  {  
    PORTB= 0b00000000;
  }
  
 
 
  /* Function name : forward
  *  Input : NONE
  *  Output : NONE
  *  Logic:Makes the bot move forward 
  * Example call:forward();
  */
  void forward()
  {
   PORTB= 0b00110011;

  }  
   
  
  
   /* Function name : timer5_init
   *  Input : NONE
   *  Output : NONE
   *  Logic: Timer 5 initialized in PWM mode for velocity control
   * Example call:timer5_init();
   */
   void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}



/* Function name : velocity
*  Input : unsigned char left_motor, unsigned char right_motor
*  Output : NONE
*  Logic:Function for velocity control
* Example call:velocity(unsigned char left_motor, unsigned char right_motor);
*/
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

	 
	 
//***************************************************************************************************************//
//______________________________________________________________________________________________________________//
//The following part is used for ADC conversion for reading sensor values//
//*************************************************************************************************************//
//____________________________________________________________________________________________________________//

/* Function name : adc_init
*  Input : NONE
*  Output : NONE
*  Logic: initialize registers ADCSRA , ADCSRB, ADMUX, ACSR ,ADCSRA  for ADC conversion
*  Example call:adc_init();
*/
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//V-reference=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

/* Function name : ADC_Conversion
*  Input : unsigned char Ch
*  Output : 'a' Digital value of analog sensor reading
*  Logic: Takes in value of channel 'ch' and returns digital value of analog sensor reading
* Example call:ADC_Conversion(unsigned char Ch);
*/
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}



//___________________________________________________________________________________________________________//

/* Function name : devices_init
*  Input : NONE
*  Output : NONE
*  Logic: initializes all devices and clears and enables global interrupts
* Example call: devices_init();
*/
void devices_init()
{
	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	sei();   //Enables the global interrupts
}	 
	 
	 
//**************************************************************************************************************//
//______________________________________________________________________________________________________________//
/* Function name : main
*  Input : NONE
*  Output : NONE
*  Logic: First all devices and configuration is done then the code reads signal value from xbee and perform
          operations accordingly  
* Example call: main();
*/
int main()
{
   devices_init();
    while (1) 
    {
		data = UDR0; 				//making copy of data from UDR0 in 'data' variable

		UDR0 = data; 				//echo data back to PC

		if(data == 0x02)            //ASCII value of 2 
		{                           //Condition :  Required node is reached , now bot should dispatch aid
			stop();
			buzzer_on_off(); 
			striker_servo();
		}
		else if(data == 0x01)     //ASCII value of 2
		{                         //Condition :  All coins are served and bot is at capital, so it should stop
			stop();
		}
		else                      ////Condition : move along white line
		{
          Left_white_line = ADC_Conversion(3);	//Getting data of Left White Line Sensor
          Center_white_line = ADC_Conversion(2);	//Getting data of Center White Line Sensor
          Right_white_line = ADC_Conversion(1);	//Getting data of Right White Line  Sensor

          flag=0;

			   if(Center_white_line>0x10)
			  {
				  flag=1;
				  forward();
				  velocity(125,100);      /*our one motor is inherently weak so to make
				                          it move forward it's speed needs to be increased
										  greater than other motor*/
			  }

			  if((Left_white_line>0x10) && (flag==0))
			  {
				  flag=1;
				  forward();
				  velocity(150,100);
			  }

			  if((Right_white_line>0x10) && (flag==0))
			  {
				  flag=1;
				  forward();
				  velocity(162,82);
			  }

			  if(Center_white_line>0x10 && Left_white_line>0x10 && Right_white_line>0x10)
			  {
				  forward();
				  velocity(0,0);
			  }
		}
    }
}

