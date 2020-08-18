/*
 * Util.h
 *
 * Created: 01.05.2020 14:19:29ccc
 *  Author: Shlum
 */


#ifndef UTIL_H
#define UTIL_H
/*
#ifdef __cplusplus
  extern "C" {
#endif
*/
#include <inttypes.h>

#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

//#include "UART/uart.h"
//#include "Timer.h"

//---- cImportant startup settings...   ----------------------------------------------------------------------------------------------------------------------------------------------------------------------



  //--------- util macros ------------------------------------------
  #define BIT(x) (1 << (x))
  #define GET_BIT(SFR, BIT_NUM) ((SFR) &   (BIT(BIT_NUM))) // get 1 bit from Register SFR
  #define SET_BIT(SFR, BIT_NUM) ((SFR) |=  (BIT(BIT_NUM))) // set 1 bit (to 1) in Register SFR
  #define CLR_BIT(SFR, BIT_NUM) ((SFR) &= ~(BIT(BIT_NUM))) // clear 1 bit (to 0) in Register SFR
  #define TGL_BIT(SFR, BIT_NUM) ((SFR) ^=  (BIT(BIT_NUM))) // toggle 1 bit in Register SFR

  #define SET_BIT_MASK(PORT,MASK) ((PORT) |= (MASK))
  #define CLR_BIT_MASK(PORT,MASK) ((PORT) &= (~MASK))

  #define min(x,y) ((x)<(y)?(x):(y))
  #define max(x,y) ((x)>(y)?(x):(y))

  // union erlaubt einen effektiven, separaten Zugriff auf Teile der Variable
  typedef union
  {
    uint16_t i16;// all
    struct
    {
      uint8_t low;   // low
      uint8_t high;  // high
    };
  } convert16to8;


#ifdef DEBUG
  #define _DELAY_MS(x)
#else
  #define _DELAY_MS(x) _delay_ms(x)
#endif // _DEBUG


  /*
#ifdef __cplusplus
  }
#endif
*/
#endif /* UTIL_H */