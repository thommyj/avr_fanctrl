#include <avr/io.h>
#include <avr/interrupt.h>

#define EDGE_IDLE     (0)
#define EDGE_START    (9)

#define STARTBIT (10)
#define STOPBIT   (1)
#define IDLE      (0)

uint8_t volatile mcuTime = 0;
uint16_t volatile tickCnt = 0;
uint8_t setPoint;
uint8_t duty = 0xFF;
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
	OCR0A = duty;

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
	static uint16_t lastEdgeTick;

	PINB |= (1 << PB3);
	
	if(edgeStatus == EDGE_START) {
		/* this is the first edge */
		startEdgeTick = tickCnt;
		lastEdgeTick = tickCnt;
		edgeStatus--;
	}else{	
		/*
	  	 * with min 750 rpm and max 3000 rpm, there should be
	 	 * between 750 -180 ~ 1024 - 150 ticks between two edges
	 	 */
		uint16_t tmp = (tickCnt - lastEdgeTick);
		if (tmp < 150 || tmp > 1024) {
			/* somethings not right, remove sample */
			startEdgeTick += tmp;
		} else {
			edgeStatus--;
			timeBetweenEdges = (tickCnt - startEdgeTick);
		}
		lastEdgeTick = tickCnt;
	}
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

void updateSetPoint (void)
{
	int i;

	setPoint = 0;
	for(i = 0; i < 4; i++) {
		setPoint += pot[i] >> 2;
	}
}

/*
 * 1.9.6
 */
void updateDutyCycle (void)
{
	static int16_t duty2 = 0x3FFF;
	int16_t adjust = (setPoint - actualValue) << 3;
	duty2 += adjust;
	if(duty2 < (8 << 6)) {
		duty2 = (8 << 6);
	} else if (duty2 > (255 << 6)) {
		duty2 = 0x3FFF;
	}

	duty = (uint8_t)(duty2 >> 6);

	uart_putuint8(duty);
	uart_putuint8(setPoint);
	uart_putuint8(actualValue);
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
				/* ticks are between 150 to 900 per edge, 8 edges so
				 * 1200 < timeBetweenEdges < 7200, scale down and shift
				 * to 0 -255. (7200-1200)/256~3 ~> t/4+t/16
				 */
				uint16_t tmp = (timeBetweenEdges - 1200) >> 3;
				tmp = (tmp>>2) + (tmp>>4);
				if (tmp > 255) {
					actualValue = 0;
				} else {
					actualValue = 255 - (uint8_t)tmp;
				}
				GIFR  |= (1 << INTF0);
				GIMSK |= (1 << INT0);
				edgeStatus = EDGE_START;
			} else {
				uart_putc('c');
			}
		}
	}
	return 0;
}
