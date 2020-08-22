/*
 * WellPumpCtrl.c
 *
 * Created: 15.08.2020 20:47:48
 * Author : ille
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <string.h>



#include "uart/uart.h"
#include "util.h"

#define PO_PUMP1  PD4
#define PO_PUMP2  PD5

#define PO_LED_BLUE  PB0
#define PO_LED_RED   PB1
#define PO_LED_WHITE PB2
#define PO_LED_GREEN PB3

#define PI_DISPL_BTN PB5
#define PO_DISPL_DTA PB6
#define PO_DISPL_CLK PB7

#define PO_W_SENS1   PC0
#define PI_W_SENS2   PC1

#define AIN_POTI_AMOUNT    PA0  //ADC0
#define AIN_POTI_INTERVALL PA1  //ADC1

#define CH_AMOUNT   0
#define CH_INTERVAL 1

#define CFG_DDRA 0
#define CFG_DDRB ( BIT(PO_LED_BLUE) | BIT(PO_LED_RED) | BIT(PO_LED_WHITE) | BIT(PO_LED_GREEN) | BIT(PO_DISPL_DTA) | BIT(PO_DISPL_CLK) )
#define CFG_DDRC ( BIT(PO_W_SENS1) | BIT(PI_W_SENS2) )  // init water sensor as output, low level
#define CFG_DDRD ( BIT(PO_PUMP1) | BIT(PO_PUMP2) )

  //Konfiguration der Pull-Ups für Eingänge
#define CFG_PORTA 0
#define CFG_PORTB BIT(PI_DISPL_BTN)  //pullup for button
#define CFG_PORTC 0
#define CFG_PORTD 0





#define ADMUX_PRESET  ( BIT(REFS0) | BIT(ADLAR) )  // preset of ADMUX register to left align ADC result and use only 8 MSB, VACC as VREF



void Init();
void ShowStart();
void SendDisplayData(uint8_t dta[5]);
void DisplayBCD(uint8_t bcd[3]);
void DisplayInt(int16_t i);

void StartPump();
void StopPump();
uint8_t CheckWater();
void SchowPumping();
uint16_t GetAmountADC();
uint16_t GetIntervalADC();



const uint8_t lookUpDigi1[16];


volatile uint8_t gAdcVal[2]={0};   // filled automatically in ADC ISR


uint16_t gX=1;
volatile uint16_t gPumpOnSeconds=0;    // pump seconds ON time (10..10000); set when gIrrigateInrerval is 0 (according to AIN_POTI_AMOUNT); clear in TIM1 ISR
volatile uint16_t gHours=0;            //used in TIM1 ISR (0..(24x7)-1)
volatile uint8_t  gIrrigateInrerval=0; //(according to AIN_POTI_INTERVAL);

typedef enum {stInit, stIdle, stPumping} TStatus;
TStatus gSatus;


int main(void)
{
   uint8_t loop=0, lastBtn=0xff, x;

   Init();


   ShowStart();



   //  uart_init(UART_BAUD_SELECT(19200, F_CPU));
   uart_init(UART_BAUD_SELECT(28800, F_CPU));  // the best suitable baud rate @ 15MHz

   uart_puts_P("\r\n\n\n\n\n\n\n\n\n************* Starting Flower Pot Irrigation  **************\r\n");
   uart_puts_P("Build on " __DATE__ " at " __TIME__"\r\n");
   uart_write_P("\r\n UBRRL=%u, MCUCSR=0x%02x\r\n", UBRRL, MCUCSR);

   SET_BIT(ADCSRA, ADSC);  // start first conversion

   while(1)
   {
      x = GET_BIT(PINB, PI_DISPL_BTN);
      if(lastBtn != x)
      {
         lastBtn = x;
         if(lastBtn==0)  // button is pressed
         {
            TGL_BIT(PORTB, PO_LED_WHITE);
            gX *= 2;
         }

      }

      DisplayInt(gAdcVal[0]*4);

      if(CheckWater())
        CLR_BIT(PORTB, PO_LED_RED);
      else
        SET_BIT(PORTB, PO_LED_RED);


      if(++loop == 10)
      {
         loop=0;

         uart_write_P("\r\nADC: CH0=%u, ch1=%u", gAdcVal[0], gAdcVal[1]);
         TGL_BIT(PORTB, PO_LED_BLUE);
         TGL_BIT(PORTD, PO_PUMP1);
      }
      _delay_ms(100);
   }  // while(1)
}  // main()


//--------------------------------------------------------------------------
//  void Init(void)
//--------------------------------------------------------------------------
void Init(void)
{
  //Konfiguration der Ausgänge bzw. Eingänge
  //definition erfolgt in der config.h
  DDRA = CFG_DDRA;
  DDRB = CFG_DDRB;   // wird auch für kommunikation mit ENC26C60 genutzt!!!
  DDRC = CFG_DDRC;
  DDRD = CFG_DDRD;// | MASK_LED_BEEP;

  //Konfiguration der Pull-Ups für Eingänge
  //definition erfolgt in der config.h
  PORTA=CFG_PORTA;
  PORTB=CFG_PORTB;
  PORTC=CFG_PORTC;
  PORTD=CFG_PORTD;

  // watchdag
 // wdt_reset();
 // WDTCSR=BIT(WDCE) | BIT(WDE); //enable chaneg first
//  WDTCSR=BIT(WDIE) | BIT(WDE) | BIT(WDP3) | BIT(WDP0); // enable Watchdog, timeout 8sec

//ADC
  ADCSRA = BIT(ADEN) | BIT(ADIE) | 0x7F;  // enable ADC and ADC.IRQ and pre-scaler to 128 (112 kHz sampling @ 14.3MHz clock)

//TIM1 :: 1s tick
  TCCR1A = 0;
  TCCR1B = BIT(WGM12) | BIT(CS12) | BIT(CS10); // CTC Mode, 1024 Pre-Scaler==> 13.983kHz
  OCR1A = 13983-1; //==> 1Hz tick
  SET_BIT(TIMSK, OCIE1A);
}



//---------------------------------------------------------------------------------------------------------------------------------------------------
//  void ShowStart()
//
//---------------------------------------------------------------------------------------------------------------------------------------------------
void ShowStart()
{
  CLR_BIT(PORTB, PO_LED_BLUE);
  SET_BIT(PORTB, PO_LED_RED);
  CLR_BIT(PORTB, PO_LED_WHITE);
  SET_BIT(PORTB, PO_LED_GREEN);

  _delay_ms(1000);
  for (uint8_t xx=0; xx<20; xx++)
  {
    TGL_BIT(PORTB, PO_LED_BLUE);
    TGL_BIT(PORTB, PO_LED_RED);
    TGL_BIT(PORTB, PO_LED_WHITE);
    TGL_BIT(PORTB, PO_LED_GREEN);
    _delay_ms(200);
  }

  SET_BIT(PORTB, PO_LED_BLUE);
  CLR_BIT(PORTB, PO_LED_RED);
  SET_BIT(PORTB, PO_LED_WHITE);
  CLR_BIT(PORTB, PO_LED_GREEN);

}




//-------------------------------------------------------------------------------------------
//  void DisplayInt(int16_t i)
//
//-------------------------------------------------------------------------------------------
void DisplayInt(int16_t i)
{
   uint8_t bcd[3];

   bcd[2] = i % 10;
   bcd[1] = (i % 100) / 10;
   bcd[0] = i / 100;

   DisplayBCD(bcd);
}



//-------------------------------------------------------------------------------------------
//  void DisplayBCD(uint8_t bcd[3])
//
//-------------------------------------------------------------------------------------------
void DisplayBCD(uint8_t bcd[3])
{
   uint8_t data[5]={0};  // 40 bit (35 will be needed

//Digit 1
   data[0] = lookUpDigi1[bcd[0]] ;
// Digit 2
   data[1] = lookUpDigi1[bcd[1]] << 2;
   data[2] = (lookUpDigi1[bcd[1]] >> 2) & 0x1C;
//Digit 3
   data[2] |= (lookUpDigi1[bcd[2]] << 5) ;
   data[3] = (lookUpDigi1[bcd[2]] & 0x08) >> 2 ;
   data[3] |= (lookUpDigi1[bcd[2]] & 0x70);


   SendDisplayData(data);
}




#define CLOCK(){\
   SET_BIT(PORTB, PO_DISPL_CLK);\
   _delay_us(1);\
   CLR_BIT(PORTB, PO_DISPL_CLK);\
   _delay_us(1);\
}




//-------------------------------------------------------------------------------------------
//  void SendDisplayData(uint8_t dta[])
//
//-------------------------------------------------------------------------------------------
void SendDisplayData(uint8_t dta[5])
{
  uint8_t clk, byte, bit;
  SET_BIT(PORTB, PO_DISPL_DTA);

  CLOCK();  //first of 36 clock ticks

  for (clk=0; clk<35; clk++)
  {
    CLOCK();

    byte = clk / 8;
    bit= clk % 8;

    if(GET_BIT(dta[byte], bit))
       SET_BIT(PORTB, PO_DISPL_DTA);
    else
       CLR_BIT(PORTB, PO_DISPL_DTA);
  }
}


//-------------------------------------------------------------------------------------------
//  void StartPump()
//
//-------------------------------------------------------------------------------------------
void StartPump()
{

}


//-------------------------------------------------------------------------------------------
//  void StopPump()
//
//-------------------------------------------------------------------------------------------
void StopPump()
{

}


//-------------------------------------------------------------------------------------------
//  uint8_t CheckWater()
//
//-------------------------------------------------------------------------------------------
uint8_t CheckWater()
{
   uint8_t sens;
   // initially sens1 & sens 2 are low level outputs

   CLR_BIT(DDRC, PO_W_SENS1); // switch sens1 to input ..
   SET_BIT(PORTC, PO_W_SENS1); // ..and enable pull-up

   _delay_ms(1);               // sens1 high, sens2 low

   CLR_BIT(DDRC, PI_W_SENS2); // switch sens2 to input ..
   SET_BIT(PORTC, PI_W_SENS2); // ..and enable pull-up
   CLR_BIT(PORTC, PO_W_SENS1); // disable pull-up sens1 ..
   SET_BIT(DDRC, PO_W_SENS1);  // ..and set to output

   _delay_ms(1);      // sens1 low, sens2 high

   sens = GET_BIT(PINC, PI_W_SENS2); //if high: no water
   CLR_BIT(PORTC, PI_W_SENS2); // disable pull-up sens2 ..
   SET_BIT(DDRC, PI_W_SENS2);  // ..and set to output

   return sens ? 0 : 1;
}  // uint8_t CheckWater()


//-------------------------------------------------------------------------------------------
//  void SchowPumping()
//
//-------------------------------------------------------------------------------------------
void SchowPumping()
{

}


//-------------------------------------------------------------------------------------------
//  uint16_t GetAmountADC()
//
//-------------------------------------------------------------------------------------------
uint16_t GetAmountADC()
{
   return 0;
}





//-------------------------------------------------------------------------------------------
//  uint16_t GetIntervalADC()
//
//-------------------------------------------------------------------------------------------
uint16_t GetIntervalADC()
{
   return 0;

}


ISR(TIMER1_COMPA_vect)
{
   static uint16_t seconds=0;

   seconds++;

   if(seconds==3600)
   {
      gHours++;
      seconds=0;
   }

   if(gPumpOnSeconds > 0)
   {

   }

   TGL_BIT(PORTB, PO_LED_GREEN);
}



//--------------------------------------------------------------------------
//  ADC_vect
//    ADC in self trigger mode
//    collects automatically 3 values:
//    -
//    -
//--------------------------------------------------------------------------
ISR(ADC_vect)
{
  static uint8_t ch=0;

  gAdcVal[ch]=ADCH; //ignore lower 2 bit;
  if(++ch == 2)
    ch=0;

  ADMUX= ADMUX_PRESET | ch;

  SET_BIT(ADCSRA, ADSC);  // start next conversion
}  // ISR(ADC_vect)


const uint8_t lookUpDigi1[16]=
{
/*0*/ 0b00111111,
/*1*/ 0b00000110,
/*2*/ 0b01011011,
/*3*/ 0b01001111,
/*4*/ 0b01100110,
/*5*/ 0b01101101,
/*6*/ 0b01111101,
/*7*/ 0b00000111,
/*8*/ 0b01111111,
/*9*/ 0b01101111,
/*A*/ 0b01110111,
/*B*/ 0b01111100,
/*C*/ 0b00111001,
/*D*/ 0b01011110,
/*E*/ 0b01111001,
/*F*/ 0b01110001,
};
