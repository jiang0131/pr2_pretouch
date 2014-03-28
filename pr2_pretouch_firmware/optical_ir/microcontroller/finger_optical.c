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
volatile char ch;

int main (void) 
{  
  /********* SWITCH SELECT OUTPUT ********/
  DDRD |= (1<<DDD4) | (1<<DDD5) | (1<<DDD6) | (1<<DDD7); //set PD4-7 as output for switch selects

  /**** EXTERNAL INTERRUPT *****/
  DDRD &= ~(1<<DDD2); //setup External interrupt 0 as Input (PD2) (clear bit 2)
  PORTD |= (1<<PD2); //set PD2(INT0) to high (impedance) (pull-up resistor is activated)
  EIMSK |= (1<<INT0);  //enable INT0
  EICRA |= (1<<ISC01); //INT0 triggered on falling edge

  /*********** ADC ***********/
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128 - 62.5KHz sample rate @ 8MHz 
  //ADCSRA |= (1 << ADPS2) | (1 << ADPS1); // (8M/64)
  //ADCSRA |= (1 << ADPS2) | (1 << ADPS0); // (8M/32)
  //ADCSRA |= (1 << ADPS2); // (8M/16)
  //ADCSRA |= (1 << ADPS1) | (1 << ADPS0); // (8M/8)
  ////ADCSRA |= (1 << ADPS1); // (8M/4)
  //ADCSRA |= (1 << ADPS0); // (8M/2)   

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

  ch = 1;  

  /* MAIN LOOP*/
  for(;;)  // Loop Forever 
  { 
  } 
} 

ISR(INT0_vect) {

  // Before an ADC conversion, send signal to switch to select the desired channel
  // Order: PD7->PD6->PD5->PD4 (D1->D2->D3->D4)
  PORTD |= _BV(8-ch);
  
  // Wait for the switch working and the signal going through to opamp
  //_delay_ms(2);
  _delay_us(1);//10

  // Single ADC conversion
  ADCSRA |= (1 << ADSC); //single ADC conversion

  // Wait for the ADC to finish
  //_delay_ms(1);
  _delay_us(3);//5

  // De-select the current channel
  PORTD &= ~_BV(8-ch);

  // Increment the Channel number
  ch = ch==4 ? 1 : ch+1; 
}

ISR(ADC_vect) { 
  // move the ADC value to SPI buffer
  SPDR = ADCH;
  // wait for SPI transmission finish and get data
  char idx = SPI_SlaveReceive();
  if (idx == 1) {
    // the master just requested data from the last channel
    // the next desired channel will be the first
    // reset the ch to make sure it is aligned:
    // idx: 1 4 3 2 1 4 3 2 1
    //  ch: ? 1 2 3 4 1 2 3 4
    ch = 1;
  }
} 

char SPI_SlaveReceive(void) {
  // Wait for reception complete (until SPIF is set)
  while(!(SPSR & (1<<SPIF)));
  // Return Data Register
  return SPDR;
}

