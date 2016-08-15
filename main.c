#include <avr/io.h>
#include <avr/interrupt.h>

#define EDGE_IDLE     (0)
#define EDGE_START    (5)

#define STARTBIT (10)
#define STOPBIT   (1)
#define IDLE      (0)

uint8_t volatile mcuTime = 0;
uint16_t volatile tickCnt = 0;
uint8_t setPoint;
uint8_t newOCR0A = 0xFF;
uint8_t actualValue = 0xFF;
uint8_t pot[4];
uint8_t potCnt = 0;
uint8_t uartChar = 0;
uint8_t volatile uartBusy = 0;
uint8_t volatile int0EnableTime = 0;
uint16_t volatile timeBetweenEdges = 0;
uint8_t volatile edgeStatus;


ISR(TIM0_OVF_vect)
{
	static uint8_t nextTick = 37;
	uint8_t tickCntLow = (uint8_t)++tickCnt;
	/*
	 * update setpoint at beginning of a timer fire to 
	 * avoid aliasing
	 */
	OCR0A = newOCR0A;

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
	static uint16_t startEdgeTick;

	PINB |= (1 << PB3);
	
	if(edgeStatus == EDGE_START) {
		/* this is the first edge */
		startEdgeTick = tickCnt;
	}else{	
		timeBetweenEdges = (tickCnt - startEdgeTick);
	}
	/*
	 * max rpm <3000rpm => 3000/60*2=100Hz
	 *---      --------      ---
	 *  |------|      |------|
	 *  x----^ x----^ x----^ x--
	 *
	 * so its safe to say that we shouldn't get a new
	 * interrupt earlier than 1/100*1/2*80%=4ms from now
	 *
	 */
	edgeStatus--;
	int0EnableTime = mcuTime + 4;
	GIMSK = ~(1 << INT0);
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

void uart_putnibble(uint8_t n)
{
	if (n > 0x9)
		uart_putc(n - 0xA + 'A');
	else
		uart_putc(n + '0');
}

void uart_putuint8(uint8_t i)
{
	uart_putnibble(i >> 4);
	uart_putnibble(i & 0xF);
}

void checkPot (void)
{
	if (ADCSRA & (1 << ADSC)) {
		uart_putc('d');
		return;
	}
	pot[potCnt % 4] = ADCH;
	potCnt++;
	ADCSRA |= 1 << ADSC;
}
/* avoid strange effects with too low rpm */
#define MIN_SETPOINT (0x40)
#define MAX_SETPOINT (0xFF)
void updateSetPoint (void)
{
	int i;
	/*8.8 */
	uint16_t newSetPoint = 0;

	for(i = 0; i < 4; i++) {
		newSetPoint += (pot[i] << 6);
	}

	newSetPoint = (newSetPoint >> 1) + (newSetPoint >> 2);
	newSetPoint += (MIN_SETPOINT << 8);
	setPoint = (newSetPoint >> 8);
}

#define FRACTION_BITS (7)
#define MAX_DUTY      (0xFF << FRACTION_BITS)
/* avoid to small duty, that might give the fan and rpm signal problem */
#define MIN_DUTY      (0x2  << FRACTION_BITS)
#define KP            (32) //2^5
#define MAX_E         (64)
#define MIN_E         (-64)
#define KI            (8)  //2^3
void updateDutyCycle (void)
{
	/*
	 * Values uses fixed point arithmetic to permit fractional K's and to allow
	 * I part to work properly. The duty programmed in the timer is a 8bit
	 * unsigned value though.
	 * 9 integer bits are used, which leaves 7 bits for the fractional part.
	 * 0x7f80 is max allowed value for u, since that represents 255
	 */
	static int16_t P = 0;
	static uint16_t u = 0x7f80;
	int16_t I = 0;
	int8_t e;
	int16_t tmp;

	/* 
	 * Avoid to high or low P and I by capping
	 * the error
	 */
	tmp = setPoint - actualValue;
	if (tmp > MAX_E)
		e = MAX_E; 
	else if (tmp < MIN_E)
		e = MIN_E;
	else
		e = setPoint - actualValue;
	/*
	 * u(k) = u(k-1) + deltaP(k) + deltaI(k) =>
	 * u(k) = u(k-1) + P(k) - P(k-1) + I(k) - I(k-1) =>
	 * / I(k) = Ki*e(k) + I(k-1) / =>
	 * u(k) = u(k-1) + P(k) - P(k-1) + Ki*e(k) =>
	 * / P(x) = Kp*e(x) / =>
	 * u(k) = u(k-1) + Kp*(e(k)-e(k-1)) + Ki*e(k)
	 */

	u -= P;
	P = e * KP;
	/* 
	 * max u = 0x7f80 + 64*64 = 0x8F80
	 * min u = 0x0100 - 64*64 = 0xF0FF
	 */
	u += P;

	I = e * KI;
	/*
	 * max u = 0x8F80 + 64*8 = 0x9180
	 * min u = 0xF0FF - 64*8 = 0xEEFF
	 */
	u += I;

	if (u > 0x9180 || u < MIN_DUTY)
		u = MIN_DUTY;
	else if (u > MAX_DUTY)
		u = MAX_DUTY;

	newOCR0A = (uint8_t)(u >> FRACTION_BITS);
	uart_putuint8((uint8_t)(P >> FRACTION_BITS));
	uart_putuint8((uint8_t)e);
	uart_putuint8(newOCR0A);
	uart_putc('\n');
}
	

int main (void)
{
	uint8_t currentTime = -1;

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
	 * start at 100% (OCR0A=255)
	 * enable interrupt on overflow
	 */
	TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
	OCR0A  = 0xFF;
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
	edgeStatus = EDGE_START;
	while(1) {
		while(currentTime == mcuTime) {
			/* paus work */
		}
		currentTime = mcuTime;
		
		if (int0EnableTime == currentTime && edgeStatus != EDGE_IDLE) {
			GIFR  |= (1 << INTF0);
			GIMSK |= (1 << INT0);
		}

		if ((currentTime & 0x7) == 0 && currentTime < 4*8) {
			checkPot();
		}

		if(currentTime == 128) {
			updateSetPoint();
			updateDutyCycle();
		}

		if (currentTime == 255) {
			if (edgeStatus == EDGE_IDLE) {
				/* ticks are between 170 to 1262 per edge, 4 edges so
				 * 680 < timeBetweenEdges < 5048, scale down and shift
				 * to 0 - 255. (5048 - 680)/4/256=4.27 ~ 1/(1/4 - 1/64)
				 */
				uint16_t tmp = (timeBetweenEdges - 680) >> 2;
				tmp = (tmp>>2) - (tmp>>6);
				if (tmp > 255) {
					actualValue = 0;
				} else {
					actualValue = 255 - (uint8_t)tmp;
				}
				edgeStatus = EDGE_START;
				GIFR  |= (1 << INTF0);
				GIMSK |= (1 << INT0);
			/* no pulses, perhaps to low duty to move blades */
			} else {
				actualValue = 0;
				uart_putc('c');
			}
		}
	}
	return 0;
}
