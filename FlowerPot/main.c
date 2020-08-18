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




#define AIN_POTI_AMOUNT    PA0  //ADC0
#define AIN_POTI_INTERVALL PA1  //ADC1

#define CH_AMOUNT   0
#define CH_INTERVAL 1

#define CFG_DDRA 0
#define CFG_DDRB ( BIT(PO_LED_BLUE) | BIT(PO_LED_RED) | BIT(PO_LED_WHITE) | BIT(PO_LED_GREEN) )
#define CFG_DDRC 0
#define CFG_DDRD ( BIT(PO_PUMP1) | BIT(PO_PUMP2) )

  //Konfiguration der Pull-Ups für Eingänge
#define CFG_PORTA 0
#define CFG_PORTB 0
#define CFG_PORTC 0
#define CFG_PORTD 0





#define ADMUX_PRESET  ( BIT(REFS0) | BIT(ADLAR) )  // preset of ADMUX register to left align ADC result and use only 8 MSB, VACC as VREF



void Init();
void ShowStart();


volatile uint8_t gAdcVal[2]={0};   // filled automatically in ADC ISR



int main(void)
{

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
    _delay_ms(2000);
    uart_write_P("\r\nADC: CH0=%u, ch1=%u", gAdcVal[0], gAdcVal[1]);
    TGL_BIT(PORTB, PO_LED_BLUE);

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

//TIM0 :: amount
  TCCR0 = BIT(WGM01) | BIT(CS02) | BIT(CS00); // CTC Mode, 1024 Pre-Scaler:: 14.012kHz

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


