/*
Firmware for PR2 optical fingertip sensor
TODO: transmitting ADC signal from SPI Slave (itself) to SPI Master (PicoBlaze)
microcontroller: Atmel AVR ATMega328P/168P
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
  //DDRD |= (1<<DDD7) | (1<<DDD6); //set all PD6, PD7 to LED output
  DDRD |= (1 << DDD6); // set PD6 as LED output (infrared red, 940nm)
  DDRD |= (1 << DDD7); // set PD7 as LED output (for debug)
  PORTD |=  (1<<PD6); // enable LED (always on)
  PORTD |=  (1<<PD7); // enable LED

  /**** EXTERNAL INTERRUPT *****/
  DDRD &= ~(1<<DDD2); // setup External interrupt 0 as Input (PD2) 
  PORTD |= (1<<PD2); // set PD2(INT0) to high (impedance) (pull-up resistor is activated)
  EIMSK |= (1 << INT0);  // Enable INT0
  EICRA |= (1 << ISC01); // Trigger on falling edge of INT0

  /*********** ADC ***********/
  // setup ADC conversion speed (pre-scalar)
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128 - 125KHz sample rate @ 16MHz 
  //ADCSRA |= (1 << ADPS2) | (1 << ADPS1); // (16M/64=250KHz)
  //ADCSRA |= (1 << ADPS2) | (1 << ADPS0); // (16M/32=500KHz) (Too fast..the concerted value is wrong)
  //ADCSRA |= (1 << ADPS2); // (16M/16=1Mhz)
  //ADCSRA |= (1 << ADPS1) | (1 << ADPS0); // (16M/8=2Mhz)
  //ADCSRA |= (1 << ADPS0); // ( 16M/2=8Mhz)   
  //ADCSRA |= (1 << ADPS1); // ( 16M/4=4Mhz) (this is for sound sampling)

  ADMUX |= (1 << REFS0); // Set ADC reference to AVCC 
  ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading 
  // MUX bits needed not to be changed in ADMUX when using ADC0 

  ADCSRA |= (1 << ADATE);  // Set ADC to Free-Running Mode 
  ADCSRA |= (1 << ADEN);  // Enable ADC 
  ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt 

  sei();   // Enable Global Interrupts 
  ADCSRA |= (1 << ADSC);  // Start 1st ADC for Free-Running Mode


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

// if the 8-bit ADC value is larger than the threshold, turn on LED
// else, dim the LED
void check_ADCH(void) {
    /*
    if (ADCH < 50) {
      PORTD &= ~(1<<PD7); //dim the LED
    }
    else {
      PORTD |=  (1<<PD7); //light the LED
    }
    */
    if (ADCW < 180) {
      PORTD &= ~(1<<PD7); //dim the LED
    }
    else {
      PORTD |=  (1<<PD7); //light the LED
    }
}


ISR(ADC_vect) { 
  //write SPDR
  SPDR = ADCH;
  
  // try blink once
  //PORTD &= ~(1<<PD7); //dim the LED
  //_delay_ms(10);
  //check_ADCH();
  //PORTD |=  (1<<PD7); //light the LED
  //_delay_ms(10);

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

