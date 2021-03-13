void setup() {

  // timer 1
  pinMode(11, OUTPUT);
  // timer 3
  pinMode(5, OUTPUT);
  // timer 4
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  // timer 5
  pinMode(44, OUTPUT);
  pinMode(45, OUTPUT);
  pinMode(46, OUTPUT);


  GTCCR = _BV(TSM) | _BV(PSRSYNC) | _BV(PSRASY); // halt all timers

  
  //timer 1
  // mode selection
  // mode = 10, non-inverting mode
  TCCR1A = _BV(WGM11) | _BV(COM1A1) | _BV(COM1B1) | _BV(COM1C1);
  TCCR1B = _BV(WGM13) | _BV(CS10) ;
  // top
  ICR1 = 400;
  // duty cycle
  OCR1A = 0.50 * 400;

  //timer 3
  // mode selection
  // mode = 10 phase correct PWM, inverting mode
  TCCR3A = _BV(WGM31) | _BV(COM3A0) | _BV(COM3B0) | _BV(COM3C0)| _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1);
  TCCR3B = _BV(WGM33) | _BV(CS30);
  // top
  ICR3 = 400;
  // duty cycle
  OCR3A = (0.5 + 0.02) * 400;

  // timer 4
  // mode selection
  // mode = 10, inverting mode
  TCCR4A = _BV(WGM41) | _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1);
  TCCR4B = _BV(WGM43) | _BV(CS40);
  // top
  ICR4 = 400;
  // duty cycle
  OCR4A = (0.5 - 0.05) * 400;

  //timer 5
  // mode selection
  // mode = 10 phase correct PWM, inverting mode
  TCCR5A = _BV(WGM51) | _BV(COM5A0) | _BV(COM5B0) | _BV(COM5C0)| _BV(COM5A1) | _BV(COM5B1) | _BV(COM5C1);
  TCCR5B = _BV(WGM53) | _BV(CS50);
  // top
  ICR5 = 400;
  // duty cycle
  OCR5A = 0.55 * 400;

  GTCCR = 0; // release all timers
}

void loop() {
  // put your main code here, to run repeatedly:

}
