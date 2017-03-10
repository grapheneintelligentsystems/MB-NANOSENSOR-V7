//**************************************************************************
// Name: 	main.c
// Project: NANOSENSOR
// Author: 	Michail Beliatis
//**************************************************************************

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "usbdrv.h"
#include "oddebug.h"

#define UTIL_BIN4(x)        (uchar)((0##x & 01000)/64 + (0##x & 0100)/16 + (0##x & 010)/4 + (0##x & 1))
#define UTIL_BIN8(hi, lo)   (uchar)(UTIL_BIN4(hi) * 16 + UTIL_BIN4(lo))

#define BIT_LED 3 	//LED control Port B
#define BIT_DS0 2 	//Discharge control 0 Port A 
#define BIT_DS1 4 	//Discharge control 1 Port A 
#define BIT_DS2 4 	//Discharge control 2 Port B 

#define B_ACOM0 6	//Analog comparator 0 Port A
#define B_ACOM1 7	//Analog comparator 1 Port A
#define B_ACOM2 5	//Analog comparator 2 Port A

#ifndef NULL
#define NULL    ((void *)0)
#endif

/* ------------------------------------------------------------------------- */

static uchar    reportBuffer[2];    	/* buffer for HID reports */
static uchar    idleRate;           	/* in 4 ms units */

static uchar    adcPending;

static uchar    valueBuffer[16];
static uchar    *nextDigit;
static uchar	flip = 0;
static uchar	mode = 0;				// 0 = discarge, 1 = charge for capacitor
static uchar 	channel = 0x0b; 		// Letter "H" Hydrogen
static unsigned int timerCnt;			//Counter for delay period
static unsigned int TimerDelay = 300;	// Delay period

/* ------------------------------------------------------------------------- */

PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = { /* USB report descriptor */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};
/* We use a simplifed keyboard report descriptor which does not support the
 * boot protocol. We don't allow setting status LEDs and we only allow one
 * simultaneous key press (except modifiers). We can therefore use short
 * 2 byte input reports.
 * The report descriptor has been created with usb.org's "HID Descriptor Tool"
 * which can be downloaded from http://www.usb.org/developers/hidpage/.
 * Redundant entries (such as LOGICAL_MINIMUM and USAGE_PAGE) have been omitted
 * for the second INPUT item.
 */

/* Keyboard usage values, see usb.org's HID-usage-tables document, chapter
 * 10 Keyboard/Keypad Page for more codes.
 */
#define MOD_CONTROL_LEFT    (1<<0)
#define MOD_SHIFT_LEFT      (1<<1)
#define MOD_ALT_LEFT        (1<<2)
#define MOD_GUI_LEFT        (1<<3)
#define MOD_CONTROL_RIGHT   (1<<4)
#define MOD_SHIFT_RIGHT     (1<<5)
#define MOD_ALT_RIGHT       (1<<6)
#define MOD_GUI_RIGHT       (1<<7)

#define KEY_1       30
#define KEY_2       31
#define KEY_3       32
#define KEY_4       33
#define KEY_5       34
#define KEY_6       35
#define KEY_7       36
#define KEY_8       37
#define KEY_9       38
#define KEY_0       39
#define KEY_RETURN  40

/* ------------------------------------------------------------------------- */

static void buildReport(void)
{
uchar   key = 0;

    if(nextDigit != NULL){
        key = *nextDigit;
    }
    reportBuffer[0] = 0;    /* no modifiers */
    reportBuffer[1] = key;
}

static void evaluateADC(unsigned int value)
{
uchar   digit;

    //value += value + (value >> 1);  /* value = value * 2.5 for output in mV */
	//value = (0.86 * value) - 232;
	//value = value - 232;
	//value = ((value >> 1) + (value >> 2)) - 179;
	nextDigit = &valueBuffer[sizeof(valueBuffer)];
    *--nextDigit = 0xff;/* terminate with 0xff */
    *--nextDigit = 0;
    *--nextDigit = KEY_RETURN;
	*--nextDigit = channel;
    do{
        digit = value % 10;
        value /= 10;
        *--nextDigit = 0;
        if(digit == 0){
            *--nextDigit = KEY_0;
        }else{
            *--nextDigit = KEY_1 - 1 + digit;
        }
    }while(value != 0);
}

static void HWInit(void)
{
    PORTB &= ~(1 << BIT_LED);   //LED off
	DDRB |= 1 << BIT_LED;		//Port B Bit 3 output for LED control
	/*
	PORTA &= ~(1 << BIT_DS0);   //Sensor 2 grounded for discharge
	DDRA |= 1 << BIT_DS0;		//Port A Bit 2 output for discarge control 0
	PORTA &= ~(1 << BIT_DS1);   //Sensor 1 grounded for discharge
	DDRA |= 1 << BIT_DS1;		//Port A Bit 4 output for discarge control 1
	PORTB &= ~(1 << BIT_DS2);   //Sensor 3 grounded for discharge
	DDRB |= 1 << BIT_DS2;		//Port B Bit 4 output for discarge control 2
	*/

	PORTA &= ~(1 << B_ACOM0);   //Sensor 2 Analog comparator 0 high impedance
	DDRA &= ~(1 << B_ACOM0);	//Port A Bit 6 input for Analog comparator 0
	PORTA &= ~(1 << B_ACOM1);   //Sensor 1 Analog comparator 1 high impedance
	DDRA &= ~(1 << B_ACOM1);	//Port A Bit 7 input for Analog comparator 1
	PORTA &= ~(1 << B_ACOM2);   //Sensor 3 Analog comparator 2 high impedance
	DDRA &= ~(1 << B_ACOM2);	//Port A Bit 5 input for Analog comparator 2

}

static void adcPoll(unsigned int NameADC)
{
unsigned int valueADC;

    if(adcPending && !(ADCSRA & (1 << ADSC))){
        adcPending = 0;

		valueADC = ADCL;
		valueADC |= ((unsigned int)ADCH << 8);


		ADCSRA &= ~ (1 << ADSC);	//DISENABLE ADC 

		if(NameADC == 0){
		//switch (NameADC)
		//{
		//case 0: 					//Letter "y" Ydrogono SENSOR 1
		ADMUX = 0b10000101;			// REF1,REF0 Vref= 2.5V, MUX4:0 ADC5:000101
		ADCSRB = 0b00010000;		// REF2 Vref= 2.5V, MUX5:0
		valueADC += valueADC + (valueADC >> 1);  // value = value * 2.5 for output in mV
		//break;
		}else if(NameADC == 1){
		//case 1: 					//Letter "h" Hydrogen SENSOR 2
		ADMUX = 0b10000100;			// REF1,REF0 Vref= 2.5V, MUX4:0 ADC4:000100
		ADCSRB = 0b00010000;		// REF2 Vref= 2.5V, MUX5:0
		valueADC += valueADC + (valueADC >> 1);  // value = value * 2.5 for output in mV
		//break;
		}else if(NameADC == 2){
		//case 2: 					//Letter "m" Moisture (RH) SENSOR 3
		ADMUX = 0b10011111;			// REF1,REF0 Vref= 1.1V, MUX4:0 Int Temp ADC11:111111  
		ADCSRB = 0b00001000;		// REF2 Vref= 1.1V, MUX5:1
		valueADC += valueADC + (valueADC >> 1);  // value = value * 2.5 for output in mV
		//break;
		}else if(NameADC == 3){
		//case 3: 					//Letter "t" Temperature Internal
		ADMUX = 0b10000110;			// REF1,REF0 Vref= 2.5V, MUX4:0 ADC6:000110
		ADCSRB = 0b00010000;		// REF2 Vref= 2.5V, MUX5:0
		valueADC = ((valueADC >> 1) + (valueADC >> 2)) - 179; //Internal Temperature in C
		//break;
		}else {
		//default:
		//return 0;         
		//break;
		}

		ADCSRA |= (1 << ADSC);  	//ENABLE ADC 

		evaluateADC(valueADC);
    }
}

static void timerPoll(uchar NameC)
{

    if(TIFR & (1 << TOV1)){
        TIFR = (1 << TOV1); 			//Clear overflow
        if(++timerCnt >= TimerDelay){ 	//Check for end of pseudorandom delay

            timerCnt = 0;				//RESET OVERRFLOW CNT

			adcPending = 1;

            ADCSRA |= (1 << ADSC);  	//start next conversion
			flip = 1 - flip;

			if(nextDigit == NULL){
				channel = NameC; 		//Change channer			
				mode = 1;				//START NEW CHARGE CYCLE
			}
        }
    }
}

static void compaPoll(unsigned int NameV)
{
unsigned int CAPV;
	/*
	if(ACSRA & (1 << ACO)){

			PORTA |= 1 << BIT_LED;      //LED on
	}else{
			PORTA &= ~(1 << BIT_LED);   //LED off
	}
	*/

	if(mode){							//CHARGE MODE

			PORTA &= ~(1 << NameV);   	//PA4 INPUT WITHOUT PULLUP (HIGH Z) (0b00000000)
			DDRA &= ~(1 << NameV);		//ALL INPUTS APART FROM CHARGE CONTROL BIT (0b00010000)
			//TCCR0B = 0b00000001;		//START TIMER1 rising edge trigger, no prescale 16bit timer so overflow = 16.5M/65536 = 252Hz = 1/F = 0.0039sec
			//TCCR0B = 0b00000010;		//START TIMER1 rising edge trigger, 8 prescale 16bit timer so overflow = (16.5M/8)/65536 = 2062500/65536 = 31.5Hz = 1/F = 0.03sec 
			//TCCR0B = 0b00000011;		//START TIMER1 rising edge trigger, 64 prescale 16bit timer so overflow = (16.5M/64)/65536 = 257812.5/65536 = 3.9Hz = 1/F = 0.25sec
			//TCCR0B = 0b00000100;		//START TIMER1 rising edge trigger, 256 prescale 16bit timer so overflow = (16.5M/256)/65536 =64453/65536 = 0.98Hz = 1/F = 1.02sec
			//TCCR0B = 0b00000101;		//START TIMER1 rising edge trigger, 1024 prescale 16bit timer so overflow = (16.5M/1024)/65536 =16113/65536 = 0.246Hz = 1/F = 4.06sec 

			if((TIFR & (1 << ICF0)) && !(ACSRA & (1 << ACO))){
				
				//PORTB |= 1 << BIT_LED; 	//LED on

				CAPV = OCR0B;			//Read 16bit value from timer
				CAPV |= ((unsigned int)OCR0A << 8);

				evaluateADC(CAPV);
				TIFR = (1 << ICF0);		//RESET ICF0
				//TCCR0B = 0b00000000;	//STOP TIMER1
				TCNT0H = 0;				//RESET TIMER1
				TCNT0L = 0;				//RESET TIMER1
				mode = 0;				//Start new discharge cycle
				//PORTA &= ~(1 << NameV);		//PA4 LOW for discharge 0b00000000;
				//DDRA |= 1 << NameV;       	//PA4 OUTPUT 0b00000100;
			}

	}else{								//DISCHARGE MODE

			PORTA &= ~(1 << NameV);		//PA4 LOW for discharge 0b00000000;
			DDRA |= 1 << NameV;       	//PA4 OUTPUT 0b00000100;
			//TCCR0B = 0b00000000;		//STOP TIMER1
			OCR0A = 0;					//RESET TIME STAMP
			OCR0B = 0;					//RESET TIME STAMP
			//TCNT0H = 0;					//RESET TIMER1
			//TCNT0L = 0;					//RESET TIMER1

	}

}

/* ------------------------------------------------------------------------- */

static void compaInit(void)
{

    ACSRA = 0b01000011;		//1.1V ref at positive input, interapt on rising edge
	ACSRB = 0b00000000;		//AIN1 input at negative input
}
/* ------------------------------------------------------------------------- */

static void timerInit(void)
{
    TCCR1B = 0b00001001; 	/* select clock: 16.5M/1k -> overflow rate = 16.5k/256k = 62.94 Hz */
}

static void timer16Init(void)
{
	TCNT0H = 0; 
	TCNT0L = 0; 
	OCR0A = 0;
	OCR0B = 0;
    TCCR0A = 0b11111000;	//16bit mode, input capture mode, rising edge, with noise canseler, input from analog comparator
	//TCCR0B = 0b00000000;	//rising edge trigger, STOP    
	//TCCR0B = 0b00000001;		//START TIMER1 rising edge trigger, no prescale 16bit timer so overflow = 16.5M/65536 = 252Hz = 1/F = 0.0039sec
	TCCR0B = 0b00000010;		//START TIMER1 rising edge trigger, 8 prescale 16bit timer so overflow = (16.5M/8)/65536 = 2062500/65536 = 31.5Hz = 1/F = 0.03sec 
	//TCCR0B = 0b00000011;		//START TIMER1 rising edge trigger, 64 prescale 16bit timer so overflow = (16.5M/64)/65536 = 257812.5/65536 = 3.9Hz = 1/F = 0.25sec
	//TCCR0B = 0b00000100;		//START TIMER1 rising edge trigger, 256 prescale 16bit timer so overflow = (16.5M/256)/65536 =64453/65536 = 0.98Hz = 1/F = 1.02sec
	//TCCR0B = 0b00000101;		//START TIMER1 rising edge trigger, 1024 prescale 16bit timer so overflow = (16.5M/1024)/65536 =16113/65536 = 0.246Hz = 1/F = 4.06sec 

}

static void adcInit(void)
{
    //ADMUX = 0b10011111;		// Vref= 1.1V, measure internal temp sens 
	ADMUX = 0b10000110;		// REF1,REF0 Vref= 2.5V, MUX4:0 ADC6:000110, ADC5:000101, ADC4:000100, Int Temp ADC11:111111  
	ADCSRA = 0b10000111;	// enable ADC, not free running, interrupt disable, rate = 1/128 
	ADCSRB = 0b00010000;	// REF2 Vref= 2.5V, MUX5
}

/* ------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* ------------------------------------------------------------------------- */

uchar	usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    usbMsgPtr = reportBuffer;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
            buildReport();
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
	return 0;
}

/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
static void calibrateOscillator(void)
{
uchar       step = 128;
uchar       trialValue = 0, optimumValue;
int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

    /* do a binary search: */
    do{
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    /* proportional to current real frequency */
        if(x < targetValue)             /* frequency still too low */
            trialValue += step;
        step >>= 1;
    }while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; /* this is certainly far away from optimum */
    for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/

void    usbEventResetReady(void)
{
    calibrateOscillator();
    eeprom_write_byte(0, OSCCAL);   /* store the calibrated value in EEPROM */
}

/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */

int main(void)
{
uchar   i;
uchar   calibrationValue;


	HWInit();

    calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
    if(calibrationValue != 0xff){
        OSCCAL = calibrationValue;
    }
    odDebugInit();
    usbDeviceDisconnect();
    for(i=0;i<20;i++){  /* 300 ms disconnect */
        _delay_ms(15);
    }
    usbDeviceConnect();
    wdt_enable(WDTO_1S);
    timerInit();
	//timer16Init();
	//compaInit();
   	adcInit();
    usbInit();
    sei();
    for(;;){    /* main event loop */
        wdt_reset();
        usbPoll();
        if(usbInterruptIsReady() && nextDigit != NULL){ /* we can send another key */
            buildReport();
            usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
            if(*++nextDigit == 0xff)    /* this was terminator character */
                nextDigit = NULL;
        }
		
		switch (channel)
		{
		case 0x1C: 			//Letter "y" Ydrogono SENSOR 1
		//compaPoll(BIT_DS1);
		//if (mode == 0){
		adcPoll(0);
		timerPoll(0x0b);	//Letter "h" Hydrogen
		//}
		break;

		case 0x0b: 			//Letter "h" Hydrogen SENSOR 2
		adcPoll(1);
		timerPoll(0x10);	//Letter "m" Moisture (RH)
		break;

		case 0x10: 			//Letter "m" Moisture (RH) SENSOR 3
		adcPoll(2);
		timerPoll(0x17);	//Letter "t" Temperature
		break;

		case 0x17: 			//Letter "t" Temperature Internal
		adcPoll(3);
		timerPoll(0x1C);	//Letter "y" Ydrogono
		break;

		default:
		return 0;         
		break;
		}
		
		if(flip){
			PORTB |= 1 << BIT_LED;      //LED on
		}else{
			PORTB &= ~(1 << BIT_LED);   //LED off
		}
		
    }
    return 0;
}
