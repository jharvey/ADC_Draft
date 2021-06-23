#define LEDPRIO          (tprio_t)140    /**< @brief Normal priority.    */

// jhprintf parameters
#define MAX_FILLER 11
#define FLOAT_PRECISION 9
#define CHPRINTF_USE_FLOAT          FALSE

#include <stdio.h>
#include <string.h>
#include <ch.h>
#include <hal.h>
//#include <chprintf.h>
#include "shell.h"
#include <math.h>
#include "stm32F407xx.h"

//#include "./cfg/usbcfg.h"

// TODO change this in ADC usage AKA delete this once ADC is better understood.
static int32_t mean;

// printf functions
int jhvprintf(const char *fmt, va_list ap);
int jhprintf(const char *fmt, ...);


int iDebug = 0; // TODO remove this debug stuff





static float lastvalue;

#define MY_NUM_CH                                              1
#define MY_SAMPLING_NUMBER                                    10

static adcsample_t sample_buff[MY_NUM_CH * MY_SAMPLING_NUMBER];
/*
 * ADC conversion group.
 * Mode:        Linear buffer, 10 samples of 1 channel, SW triggered.
 * Channels:    IN8
 */
//static const ADCConversionGroup my_conversion_group = {
//    .circular           = FALSE,
//    .num_channels       = 1,
//    .end_cb             = NULL,
//    .error_cb           = NULL,
//    /* HW dependent part below */
//    .cr1                = 0,
//    .cr2                = ADC_CR2_SWSTART,
//    // sample times for channels 10...18
//    .smpr1 = 0,
//        //ADC_SMPR1_SMP_SENSOR(ADC_SAMPLE_144),   /* input16 - temperature sensor input on STM32F4xx */
//    .smpr2 =
//        ADC_SMPR2_SMP_AN8(ADC_SAMPLE_144),      /* input8 - analog input on STM32F4xx */
//    .htr = 0,
//    .ltr = 0,
//    .sqr1 = 0,
//    .sqr2 = 0,
//    .sqr3 = ADC_SQR3_SQ1_N(8),
//};


static const ADCConversionGroup my_conversion_group = {
  FALSE,                            /*NOT CIRCULAR*/
  MY_NUM_CH,                        /*NUMB OF CH*/
  NULL,                             /*NO ADC CALLBACK*/
  NULL,                             /*NO ADC ERROR CALLBACK*/
  0,                                /* CR1 */
  ADC_CR2_SWSTART,                  /* CR2 */
  0,                                /* SMPR1 */
  ADC_SMPR2_SMP_AN8(ADC_SAMPLE_144),/* SMPR2 */
  0,                                /* HTR */
  0,                                /* LTR */
  ADC_SQR1_NUM_CH(MY_NUM_CH),       /* SQR1 */
  0,                                /* SQR2 */
  ADC_SQR3_SQ1_N (ADC_CHANNEL_IN8)  /* SQR3 */
};


/*===========================================================================*/
/* Common functions                                                          */
/*===========================================================================*/
/*
 * Retrieve the integer part of value
 */
static int32_t ftomod(float value){
  if (value >= 0)
    return (int32_t) value;
  else
    return (int32_t) -1 * value;
}
/*
 * Retrieve the decimal part of value
 */
static uint32_t ftodp(float value) {
  if (value >= 0)
    return (uint32_t) ((value - ftomod (value)) * 1000);
  else
    return (uint32_t) ((-value - ftomod (value)) * 1000);
}





// Function call used in jhprintf function
static char *jh_long_to_string_with_divisor(char *p,
                                         long num,
                                         unsigned radix,
                                         long divisor) {
  int i;
  char *q;
  long l, ll;

  l = num;
  if (divisor == 0) {
    ll = num;
  } else {
    ll = divisor;
  }

  q = p + MAX_FILLER;
  do {
    i = (int)(l % radix);
    i += '0';
    if (i > '9') {
      i += 'A' - '0' - 10;
    }
    *--q = i;
    l /= radix;
  } while ((ll /= radix) != 0);

  i = (int)(p + MAX_FILLER - q);
  do
    *p++ = *q++;
  while (--i);

  return p;
}

// A sub function call used in jhprintf function
static char *jh_ltoa(char *p, long num, unsigned radix) {

  return jh_long_to_string_with_divisor(p, num, radix, 0);
}



/*#if CHPRINTF_USE_FLOAT
static const long pow10[FLOAT_PRECISION] = {
    10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000
};

static char *ftoa(char *p, double num, unsigned long precision) {
  long l;

  if ((precision == 0) || (precision > FLOAT_PRECISION)) {
    precision = FLOAT_PRECISION;
  }
  precision = pow10[precision - 1];

  l = (long)num;
  p = jh_long_to_string_with_divisor(p, l, 10, 0);
  *p++ = '.';
  l = (long)((num - l) * precision);

  return jh_long_to_string_with_divisor(p, l, 10, precision / 10);
}
#endif*/

/**
 * @brief   System formatted output function.
 * @details This function implements a minimal @p vprintf()-like functionality
 *          The general parameters format is: %[-][width|*][.precision|*][l|L]p.
 *          The following parameter types (p) are supported:
 *          - <b>x</b> hexadecimal integer.
 *          - <b>X</b> hexadecimal long.
 *          - <b>o</b> octal integer.
 *          - <b>O</b> octal long.
 *          - <b>d</b> decimal signed integer.
 *          - <b>D</b> decimal signed long.
 *          - <b>u</b> decimal unsigned integer.
 *          - <b>U</b> decimal unsigned long.
 *          - <b>c</b> character.
 *          - <b>s</b> string.
 *          .
 *
 * @param[in] fmt       formatting string
 * @param[in] ap        list of parameters
 * @return              The number of bytes that would have been
 *                      written to @p chp if no stream error occurs
 *
 * @api
 */
int jhvprintf(const char *fmt, va_list ap) {
  char *p, *s, c, filler;
  int i, precision, width;
  int n = 0;
  bool is_long, left_align, do_sign;
  long l;
/*#if CHPRINTF_USE_FLOAT
  float f;
  char tmpbuf[2*MAX_FILLER + 1];
#else*/
  char tmpbuf[MAX_FILLER + 1];
//#endif

  while (true) {
    c = *fmt++;
    if (c == 0) {
      return n;
    }

    if (c != '%') {
      //streamPut(chp, (uint8_t)c);
      ITM_SendChar((uint8_t)c); // JH
      n++;
      continue;
    }

    p = tmpbuf;
    s = tmpbuf;

    /* Alignment mode.*/
    left_align = false;
    if (*fmt == '-') {
      fmt++;
      left_align = true;
    }

    /* Sign mode.*/
    do_sign = false;
    if (*fmt == '+') {
      fmt++;
      do_sign = true;
    }

    /* Filler mode.*/
    filler = ' ';
    if (*fmt == '0') {
      fmt++;
      filler = '0';
    }

    /* Width modifier.*/
    if ( *fmt == '*') {
      width = va_arg(ap, int);
      ++fmt;
      c = *fmt++;
    }
    else {
      width = 0;
      while (true) {
        c = *fmt++;
        if (c == 0) {
          return n;
        }
        if (c >= '0' && c <= '9') {
          c -= '0';
          width = width * 10 + c;
        }
        else {
          break;
        }
      }
    }

    /* Precision modifier.*/
    precision = 0;
    if (c == '.') {
      c = *fmt++;
      if (c == 0) {
        return n;
      }
      if (c == '*') {
        precision = va_arg(ap, int);
        c = *fmt++;
      }
      else {
        while (c >= '0' && c <= '9') {
          c -= '0';
          precision = precision * 10 + c;
          c = *fmt++;
          if (c == 0) {
            return n;
          }
        }
      }
    }

    /* Long modifier.*/
    if (c == 'l' || c == 'L') {
      is_long = true;
      c = *fmt++;
      if (c == 0) {
        return n;
      }
    }
    else {
      is_long = (c >= 'A') && (c <= 'Z');
    }

    /* Command decoding.*/
    switch (c) {
    case 'c':
      filler = ' ';
      *p++ = va_arg(ap, int);
      break;
    case 's':
      filler = ' ';
      if ((s = va_arg(ap, char *)) == 0) {
        s = "(null)";
      }
      if (precision == 0) {
        precision = 32767;
      }
      for (p = s; *p && (--precision >= 0); p++)
        ;
      break;
    case 'D':
    case 'd':
    case 'I':
    case 'i':
      if (is_long) {
        l = va_arg(ap, long);
      }
      else {
        l = va_arg(ap, int);
      }
      if (l < 0) {
        *p++ = '-';
        l = -l;
      }
      else
        if (do_sign) {
          *p++ = '+';
        }
      p = jh_ltoa(p, l, 10);
      break;
/*#if CHPRINTF_USE_FLOAT
    case 'f':
      f = (float) va_arg(ap, double);
      if (f < 0) {
        *p++ = '-';
        f = -f;
      }
      else {
        if (do_sign) {
          *p++ = '+';
        }
      }
      p = ftoa(p, f, precision);
      break;
#endif*/
    case 'X':
    case 'x':
    case 'P':
    case 'p':
      c = 16;
      goto unsigned_common;
    case 'U':
    case 'u':
      c = 10;
      goto unsigned_common;
    case 'O':
    case 'o':
      c = 8;
unsigned_common:
      if (is_long) {
        l = va_arg(ap, unsigned long);
      }
      else {
        l = va_arg(ap, unsigned int);
      }
      p = jh_ltoa(p, l, c);
      break;
    default:
      *p++ = c;
      break;
    }
    i = (int)(p - s);
    if ((width -= i) < 0) {
      width = 0;
    }
    if (left_align == false) {
      width = -width;
    }
    if (width < 0) {
      if (*s == '-' && filler == '0') {
        ITM_SendChar((uint8_t)*s++); // JH
        n++;
        i--;
      }
      do {
        ITM_SendChar((uint8_t)filler); // JH
        n++;
      } while (++width != 0);
    }
    while (--i >= 0) {
      ITM_SendChar((uint8_t)*s++); // JH
      n++;
    }

    while (width) {
      ITM_SendChar((uint8_t)filler); // JH
      n++;
      width--;
    }
  }
}


/**
 * @brief   System formatted output function.
 * @details This function implements a minimal @p printf() like functionality
 *          The general parameters format is: %[-][width|*][.precision|*][l|L]p.
 *          The following parameter types (p) are supported:
 *          - <b>x</b> hexadecimal integer.
 *          - <b>X</b> hexadecimal long.
 *          - <b>o</b> octal integer.
 *          - <b>O</b> octal long.
 *          - <b>d</b> decimal signed integer.
 *          - <b>D</b> decimal signed long.
 *          - <b>u</b> decimal unsigned integer.
 *          - <b>U</b> decimal unsigned long.
 *          - <b>c</b> character.
 *          - <b>s</b> string.
 *          .
 *
 * @param[in] fmt       formatting string
 * @return              The number of bytes that would have been
 *                      written to @p chp if no stream error occurs
 *
 * @api
 */
int jhprintf(const char *fmt, ...) {
  va_list ap;
  int formatted_bytes;

  va_start(ap, fmt);
  formatted_bytes = jhvprintf(fmt, ap);
  va_end(ap);

  return formatted_bytes;
}








/*
 * This is  a 1 second timer
 */
static THD_WORKING_AREA(waThread2, 130);
static THD_FUNCTION( Thread2, arg) {
  (void)arg;
  chRegSetThreadName("Heartbeat Blinker");
  unsigned ii;

  while (true) {
    palClearPad(GPIOE, 3U);  // Clear the pad to make LED bright
    chThdSleepMilliseconds(500);
    palSetPad(GPIOE, 3U);    // Set the pad to make LED dim
    chThdSleepMilliseconds(500);

    // TODO finish ADC

    adcConvert(&ADCD3, &my_conversion_group, sample_buff, MY_SAMPLING_NUMBER);
    mean = 0;
    for (ii = 0; ii < MY_NUM_CH * MY_SAMPLING_NUMBER; ii++) {
      mean += sample_buff[ii];
    }
    mean /= MY_NUM_CH * MY_SAMPLING_NUMBER;
    lastvalue = (float)mean * 3.3 / 4095;

    jhprintf("Last value: %d.%03.d V \n", ftomod(lastvalue), ftodp(lastvalue));

    // Debug, send some character to ITM and see if they make it.
  } // while
} // Thread2

int main(void) {
  //bool bFirstScan = true;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  // Set the analog input to be an analog input
  palSetPadMode(GPIOF, 10, PAL_MODE_INPUT_ANALOG);
  adcStart(&ADCD3, NULL);
  adcSTM32EnableTSVREFE();

  jhprintf("Just booted Version 1\n");

  // Creates the heart beat pulse thread.
  chThdCreateStatic(waThread2, sizeof(waThread2), LEDPRIO, Thread2, NULL);

  // loop forever, processing serial inputs
  while (true) {
    palClearPad(GPIOE, 5U);  // Clear the pad to make LED bright
    chThdSleepMilliseconds(500);
    palSetPad(GPIOE, 5U);    // Set the pad to make LED dim
    chThdSleepMilliseconds(500);

  }
  return 0;

} // main()
