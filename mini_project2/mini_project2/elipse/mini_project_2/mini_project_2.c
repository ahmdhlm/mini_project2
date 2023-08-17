


/*
 * Name :Ahmed Gamal Helmy
 * Diploma: 68
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
unsigned char g_seconds =0;
unsigned char g_minutes =0;
unsigned char g_hours =0;


/*********  TIMER 1  ********/
void TIMER1_Init (void){
	TIMSK |= (1<<OCIE1A); /* Enable Timer1 Compare A Interrupt */
	TCNT1=0;
	OCR1A= 1000;
	TCCR1A =(1<<FOC1A);
	TCCR1B = (1<<WGM12) | (1<<CS12) |(1<<CS10) ; //PRESCALAR = 1024
}

ISR(TIMER1_COMPA_vect){
	g_seconds++;
	if (g_seconds==60){
		g_seconds=0;
		g_minutes++;
	}
	if (g_minutes==60) {
		g_minutes=0;
		g_hours++;

	}

	if (g_hours==24) {
		g_hours=0;

	}

}




/***********INT0 ********/

void INT0_Init (void){
	DDRD &=~(1<<PD2);   //
	PORTD |=(1<<PD2); //CONFIGURE INTERNAL PULL UP
	MCUCR |=(1<<ISC01);
	GICR |=(1<< INT0);
}

ISR(INT0_vect){
	g_seconds=0; //RESET
	g_minutes=0;
	g_hours=0;
}

/*********** INT1************/



void INT1_Init (void){
	DDRD &=~(1<<PD3);
	MCUCR |=(1<<ISC11) | (1<<ISC10); // External Interrupt INT1 with raising edge
	GICR |=(1<<INT1);
}


ISR(INT1_vect){
	TCCR1B &= ~(1<<CS10) &~(1<<CS12) &~(1<<CS11);  // PUT PRESCALAR =0 TO STOP THE TIMER (NO CLOCK SOURCE )

}


/********* INT2 ************/


void INT2_Init (void){
	DDRB &=~(1<<PB2);   //
	PORTB |=(1<<PB2); //CONFIGURE INTERNAL PULL UP
	MCUCSR |=(1<< ISC2);
	GICR |=(1<< INT2);
}

ISR(INT2_vect){
	TCCR1B |=(1<<CS12) | (1<<CS10);  // PUT PRESCALAR 1024 TO RESUME TIMER


}



/************* MAIN ***********/

int main(void)
{
	SREG |=(1<<7);  //ENABLE I BIT
	DDRC |= 0x0F;  //OUTPUT OF 7 _SEGMENTS
	PORTC &=0xF0;  // SET LOW FOR 7 SEGMENT
	DDRA |=0x3F;   // OUTPUT OF ENABLES OF 7 SEGMENTS

	TIMER1_Init ();
	INT0_Init();
	INT1_Init();
	INT2_Init ();

	while(1)
	{
		PORTA =(PORTA &0xC0) | 0b00000001 ;  //C0  TO DISABLE ALL PINS IN PORTA  ORR WITH PIN1 TO ENABLE ONLY FIRST PIN
		PORTC =(PORTC &0xF0) | (g_seconds % 10);  //DISPLAY FIRST DIGIT FOR SECONDS WHICH NOT ABOVE 9
		_delay_ms(3);


		PORTA =(PORTA &0xC0) | 0b00000010 ;  //C0  TO DISABLE ALL PINS IN PORTA  ORR WITH PIN2 TO ENABLE ONLY FIRST PIN
		PORTC =(PORTC &0xF0) | (g_seconds / 10);  // display digit after get over 10
		_delay_ms(3);


		PORTA =(PORTA &0xC0) | 0b00000100 ;
		PORTC =(PORTC &0xF0) | (g_minutes % 10);
		_delay_ms(3);


		PORTA =(PORTA &0xC0) | 0b00001000 ;
		PORTC =(PORTC &0xF0) | (g_minutes / 10);
		_delay_ms(3);


		PORTA =(PORTA &0xC0) | 0b00010000 ;
		PORTC =(PORTC &0xF0) | (g_hours % 10);
		_delay_ms(3);


		PORTA =(PORTA &0xC0) | 0b00100000 ;
		PORTC =(PORTC &0xF0) | (g_hours / 10);
		_delay_ms(3);

	}

}



