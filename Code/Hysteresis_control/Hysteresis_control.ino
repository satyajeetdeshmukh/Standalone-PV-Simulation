float I_ref = 2;
float I_read = 0;
float kp = 1;
float ki = 0.5;
float kp_term = 0;
float ki_term = 0;

void setup() {

  // low side current sensor
  pinMode(A7,INPUT);

  // high side current sensor
  // pinMode(A8,INPUT);

  // timer 1
  pinMode(11, OUTPUT);
  // timer 3
  pinMode(5, OUTPUT);
  
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
  OCR3A = (0.50 + 0.02) * 400;

 
  GTCCR = 0; // release all timers
}

void take_inputs(){
  I_read = 0;
  int count = 0;
  int count_max = 5;
  while(count<count_max){
    I_read += abs((float)(analogRead(A7)/1023.0*5.0 - 2.4925)*10/count_max);
    count++;
  } 
  //Serial.println(pv_current);
  //Serial.println(pv_voltage);
  //Serial.println();
}

void loop() {
  // put your main code here, to run repeatedly:
  take_inputs();
  float e = I_read - I_ref;
  kp_term = kp * e;
  ki_term += ki * e;
  float duty = kp_term + ki_term;
  OCR1A = duty * 400;
  OCR3A = (duty + 0.02) * 400;

}
