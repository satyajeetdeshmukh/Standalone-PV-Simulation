void setup()
{
 pinMode(9, OUTPUT);  // on 2A   //Pin 9 is 2B on the mega
 pinMode(10, OUTPUT); // on 2B  //pin 10 is 2A on the mega
 pinMode(2, OUTPUT);  // on 2A   //Pin 9 is 2B on the mega
 pinMode(6, OUTPUT);  // on 2A   //Pin 9 is 2B on the mega
 pinMode(5, OUTPUT); // on 2B  //pin 10 is 2A on the mega

 TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20) ; // non-Inverting PWM,
 OCR2A = 99;
  // Fast PWM and NO prescaling 16M/1024 // your CS20/CS21 settings are a prescaler of 32.
 TCCR2B =  _BV(WGM22) | _BV(CS21);  // Fast PWM and NO prescaling 16M/1024
 OCR2B = 70;

 
 TCCR3A = _BV(COM3A1) | _BV(COM3B1) | _BV(WGM31)  ; // non-Inverting PWM,
 ICR3 = 99;
  // Fast PWM and NO prescaling 16M/1024 // your CS20/CS21 settings are a prescaler of 32.
 TCCR3B =  _BV(WGM32) | _BV(CS31) | _BV(WGM33);  // Fast PWM and NO prescaling 16M/1024
 OCR3B = 50;

TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(WGM41)  ; // non-Inverting PWM,
 ICR4 = 99;
  // Fast PWM and NO prescaling 16M/1024 // your CS20/CS21 settings are a prescaler of 32.
 TCCR4B =  _BV(WGM42) | _BV(CS41) | _BV(WGM43);  // Fast PWM and NO prescaling 16M/1024
 OCR4A = 50;

 }

void loop() {}
