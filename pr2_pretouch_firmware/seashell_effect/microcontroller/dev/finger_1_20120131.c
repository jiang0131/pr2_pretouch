/*
Firmware for transmitting ADC signal from SPI Slave (itself) to SPI Master (PicoBlaze)
microcontroller: Atmel AVR ATMega328P/168
@author: Liang-Ting Jiang
*/

#include <avr/io.h> 
#include <avr/interrupt.h> 
#include <util/delay.h>
char SPI_SlaveReceive(void);
void check_ADCH(void);

int main (void) 
{  
   /********* LED OUTPUT ********/
   DDRD |= (1<<DDD7) | (1<<DDD6); //set all PD6, PD7 to LED output for debugging

   /**** EXTERNAL INTERRUPT *****/
   DDRD &= ~(1<<DDD2); // setup External interrupt 1 as Input (PD3) (actually it's default)
   PORTD |= (1<<PD2); // set PD2(INT0) to high (impedance) (pull-up resistor is activated)
   EIMSK |= (1 << INT0);  // Enable INT0
   EICRA |= (1 << ISC01); // Trigger on falling edge of INT0

   /*********** ADC ***********/
   //ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128 - 125KHz sample rate @ 16MHz 
   //ADCSRA |= (1 << ADPS2) | (1 << ADPS1); // (16M/64=250KHz)
   //ADCSRA |= (1 << ADPS2) | (1 << ADPS0); // (16M/32=500KHz) (Too fast..the concerted value is wrong)
   //ADCSRA |= (1 << ADPS2); // (16M/16=1Mhz)
   //ADCSRA |= (1 << ADPS1) | (1 << ADPS0); // (16M/8=2Mhz)
   //ADCSRA |= (1 << ADPS0); // ( 16M/2=8Mhz)   
   ADCSRA |= (1 << ADPS1); // ( 16M/4=4Mhz)
   ADMUX |= (1 << REFS0); // Set ADC reference to AVCC 
   ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading 
   // No MUX values needed to be changed to use ADC0 
   //ADCSRA |= (1 << ADATE);  // Set ADC to Free-Running Mode 
   ADCSRA |= (1 << ADEN);  // Enable ADC 
   ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt 
   sei();   // Enable Global Interrupts 
   //ADCSRA |= (1 << ADSC);  // Start A2D Conversions 

   /********** SPI ***********/
   DDRB = (1<<DDB4); // MISO output
   SPCR = (1<<SPE); //Enable SPI 


   /* MAIN LOOP*/
   for(;;)  // Loop Forever 
   { 
   } 
} 

ISR(INT0_vect) {
  ADCSRA |= (1 << ADSC); //single ADC conversion
}

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


ISR(ADC_vect) { 
  //write SPDR
  SPDR = ADCH;
  
  //check_ADCH();

 //wait for SPI transmission finish
  SPI_SlaveReceive();

  //check_ADCH();
} 


char SPI_SlaveReceive(void) {
  // Wait for reception complete (until SPIF is set)
  while(!(SPSR & (1<<SPIF)))
  ;
  // Return Data Register
  return SPDR;
}

