#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 9600000UL
#include <util/delay.h>

uint8_t volatile mcuTime = 0;
uint16_t volatile tickCnt = 0;
uint8_t setPoint = 0; 
uint8_t pot[4] = {0x55, 0x55, 0x55, 0x55};
uint8_t potCnt = 0;
uint8_t uartChar = 0xFF;
uint8_t volatile uartBusy = 0;
uint8_t volatile enableInt0Time = 0;
uint8_t volatile timeBetweenEdges = 0;

#define STARTBIT (10)
#define STOPBIT   (1)
#define IDLE      (0)
/*
 */
ISR(TIM0_OVF_vect)
{
	static uint8_t nextTick = 37;
	uint8_t tickCntLow = (uint8_t)++tickCnt;
	/*
	 * update setpoint at beginning of a timer fire to 
	 * avoid aliasing
	 */
	OCR0A = setPoint;

	/*
	 * Fires each 256/9.6M=26.67us, so 1ms is 37.5 fires
	 * every other, count to 37 resp 38 to even out
	 */
	if (tickCntLow == nextTick) {
		mcuTime++;
		if (mcuTime & 0x1)
			nextTick = tickCntLow + 38;
		else
			nextTick = tickCntLow + 37;
	}

	/* 
 	 * 26.67 will give a baudrate at 37500, which is 
	 * close enough to 38400
	 */
	if (uartBusy == STARTBIT) {
		PORTB &= ~(1 << PB4);
		uartBusy--;
	} else if (uartBusy == STOPBIT) {
		PORTB |= (1 << PB4);
		uartBusy--;
	} else if (uartBusy != IDLE) {
		if (uartChar & 0x1) {
			PORTB |= (1 << PB4);
		} else {
			PORTB &= ~(1 << PB4);
		}
		uartChar >>= 1;
		uartBusy--;
	}
}

ISR(INT0_vect)
{
	/*
	 * max rpm <3000rpm => 3000/60*2=100Hz
	 *---      --------      ---
	 *  |------|      |------|
	 *  x----^ x----^ x----^ x--
	 *
	 * so its safe to say that we shouldn't get a new
	 * interrupt earlier than 1/100*1/2*5/6=4.17ms from now
	 *
	 */
	uint8_t rpm = PINB & (1 << PB1);
	static uint8_t  nextEdge = (1 << PB1);
	static uint16_t startEdgeTick = 0;

	/* not expected edge? must be a spurious wakeup */
	if (rpm != nextEdge) {
		enableInt0Time = 4;
		PINB |= (1 << PB3);
		nextEdge ^= (1 << PB1);
		GIMSK = ~(1 << INT0);
		/*
	 	 * with min 750 rpm and max 3000 rpm, there should be
	 	 * between 750 -180 ~ 750 - 238 ticks between two edges
		 * scale and shift this values to 0 - 256
	 	 */
		uint16_t tmp = (tickCnt - startEdgeTick);
		if (tmp <= 238) {
			timeBetweenEdges = 1;
		} else {
			tmp = (tmp - 238) >> 1;
			if (tmp > 255) {
				timeBetweenEdges = 255;
			} else {
				timeBetweenEdges = (uint8_t)tmp;
			}
		}
		startEdgeTick = tickCnt;
	}
}

/*
 * one charachter takes 10/38400~0.25ms to send
 * if already busy, putc will busywait
 */
void uart_putc(char c)
{
	while(uartBusy);
	uartChar = c;
	uartBusy = STARTBIT;
}

void uart_putuint8(uint8_t i)
{
	uint8_t i0 = i & 0xF;
	uint8_t i1 = i >> 4;

	if (i1 > 0x9)
		uart_putc(i1 - 0xA + 'A');
	else
		uart_putc(i1 + '0');

	if (i0 > 0x9)
		uart_putc(i0 -0xA + 'A');
	else
		uart_putc(i0 + '0');

}

void checkPot (void)
{
	if (ADCSRA & (1 << ADSC)) {
		//TODO: add error, should always be done
		return;
	}
	pot[potCnt % 4] = ADCH;
	potCnt++;
	ADCSRA |= 1 << ADSC;
}

void updateSetPoint (void)
{
	uint8_t averagePot = 0;
	averagePot += pot[0] >> 2;
	averagePot += pot[1] >> 2;
	averagePot += pot[2] >> 2;
	averagePot += pot[3] >> 2;
	//avoid to small values, might be problem with updates
	if (averagePot < 5) {
		setPoint = 5;
	} else {
		setPoint = averagePot; //TODO: should be rpm, not adc
	}
}

void do2msWork ()
{
	if (enableInt0Time) {

		enableInt0Time--;
		enableInt0Time--;

		if (enableInt0Time == 0) {
			GIFR  |= (1 << INTF0);
			GIMSK |= (1 << INT0);
		}
	}
}

void do8msWork ()
{
	checkPot();
	updateSetPoint();
}

int main (void)
{
	uint16_t currentTime;

	/*
	 * disable divide by 8 for systemclock
	 */
	CLKPR = 0x80;
	CLKPR = 0x0;

	/*
	 * GPIO
	 * PB0 fan    output OCOA
	 * PB1 rpm    input  GPIO int0, any logical change
	 * PB2 pot    input  ADC1
	 * PB3 led    ouput  GPIO
	 * PB4 uarttx output GPIO
	 * PB5 reset
	 */
	DDRB = (1 << PB0) | (1 << PB3) | (1 << PB4);
	PORTB = 0;
	DIDR0 = (1 << ADC1D);
	MCUCR = (1 << ISC00);
	GIMSK = (1 << INT0);
	/*
	 * timer0
	 * input = 9.6MHz
	 * output = 9.6MHz/256=37.5kHZ (CS00)
	 * Fast PWM, TOP = 0xFF mode 3 (WGM=0x3)
	 * toggle OCOA on compare (COM0A0)
	 * start at 33% (OCR0A=255/3)
	 * enable interrupt on overflow
	 */
	TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
	OCR0A  = 0x55;
	TIMSK0 = (1 << TOIE0);
	TCCR0B = (1 << CS00);

	/*
	 * adc
	 * Vcc reference
	 * left shift result (ADLAR)
	 * ADC1, PB2 (MUX = 0x1)
	 * ADC enabled (ADEN)
	 * ADCclk = 9.6M/64=150kHz (ADPS = 0x6)
	 */
	ADMUX = (1 << ADLAR) | (1 << MUX0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);

	sei();
	currentTime = mcuTime;
	while(1) {
		while(currentTime == mcuTime) {
			/* paus work */
			//printing three charaters take 1ms
			if(currentTime > (mcuTime + 1)) {
				uart_putuint8(timeBetweenEdges);
				uart_putc('\n');
			}
		}
		currentTime = mcuTime;
		if ((currentTime & 0x1) == 0) {
			do2msWork();
		}

		if ((currentTime & 0x7) == 0) {
			do8msWork();
		}
	}
	return 0;
}
