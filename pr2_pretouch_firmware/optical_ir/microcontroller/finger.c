/*
Firmware for the receiver side of PR2 optical sensor finger.
Sends ADC signal from SPI Slave (itself) to SPI Master (PicoBlaze)
microcontroller: Atmel AVR ATMega168(P)/328(P)
@Liang-Ting Jiang 01-16-2014
*/

#include <avr/io.h> 
#include <avr/interrupt.h> 
#include <util/delay.h>
char SPI_SlaveReceive(void);
//void check_ADCH(void);

volatile int CH;

int main (void) 
{  
   /********* SWITCH SELECT OUTPUT ********/
   DDRD |= (1<<DDD4) | (1<<DDD5) | (1<<DDD6) | (1<<DDD7); //set PD4-7 as output for switch selects
   CH = 7;

   /**** EXTERNAL INTERRUPT *****/
   DDRD &= ~(1<<DDD2); //setup External interrupt 0 as Input (PD2) (clear bit 2)
   PORTD |= (1<<PD2); //set PD2(INT0) to high (impedance) (pull-up resistor is activated)
   EIMSK |= (1<<INT0);  //enable INT0
   EICRA |= (1<<ISC01); //INT0 triggered on falling edge

   /*********** ADC ***********/
   ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128 - 125KHz sample rate @ 16MHz 
   //ADCSRA |= (1 << ADPS2) | (1 << ADPS1); // (16M/64=250KHz)
   //ADCSRA |= (1 << ADPS2) | (1 << ADPS0); // (16M/32=500KHz) (Too fast..the concerted value is wrong)
   //ADCSRA |= (1 << ADPS2); // (16M/16=1Mhz)
   //ADCSRA |= (1 << ADPS1) | (1 << ADPS0); // (16M/8=2Mhz)
   ////ADCSRA |= (1 << ADPS1); // ( 16M/4=4Mhz)
   //ADCSRA |= (1 << ADPS0); // ( 16M/2=8Mhz)   

   ADMUX |= (1 << REFS0); // Set ADC reference to AVCC 
   ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading 
   //No MUX values change required to use ADC0 
   //ADCSRA |= (1 << ADATE);  // Set ADC to Free-Running Mode 
   ADCSRA |= (1 << ADEN);  // Enable ADC 
   ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt 
   sei();   // Enable Global Interrupts 
   //ADCSRA |= (1 << ADSC);  // Start A2D Conversions 

   /********** SPI ***********/
   DDRB = (1<<DDB4); // MISO output
   SPCR = (1<<SPE); //Enable SPI 

   /*****fixed ADC ch (for testing only) ******/
   //PORTD |= _BV(7);
   //PORTD |= _BV(6);
   //PORTD |= _BV(5);
   //PORTD |= _BV(4);

   /* MAIN LOOP*/
   for(;;)  // Loop Forever 
   { 
   } 
} 

ISR(INT0_vect) {

  // Before an ADC conversion, send signal to switch to select the desired channel
  // Order: PD7->PD6->PD5->PD4 (D1->D2->D3->D4)
  PORTD |= _BV(CH);
  
  // Wait for the switch works and the signal going through
  //_delay_ms(2);
  _delay_us(1);//10

  // Single ADC conversion
  ADCSRA |= (1 << ADSC); //single ADC conversion

  // Wait for the ADC to finish
  //_delay_ms(1);
  _delay_us(3);//5

  // De-select the current channel
  PORTD &= ~_BV(CH);

  // Increment the Channel number
  CH = CH==4 ? 7 : CH-1; 
}

/*
void check_ADCH(void) {
    if (SPDR < 120) {
      PORTD |=  (1<<PD6);
      PORTD &= ~(1<<PD7);
    }
    else {
      PORTD |=  (1<<PD7);
      PORTD &= ~(1<<PD6);
    }
}
*/

ISR(ADC_vect) { 
  //write SPDR
  SPDR = ADCH;
  //SPDR = CH; // for testing only  
  //check_ADCH();
 //wait for SPI transmission finish
  SPI_SlaveReceive();
  //check_ADCH();
} 


char SPI_SlaveReceive(void) {
  // Wait for reception complete (until SPIF is set)
  while(!(SPSR & (1<<SPIF)));
  // Return Data Register
  return SPDR;
}

