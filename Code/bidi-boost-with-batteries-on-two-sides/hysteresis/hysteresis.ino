// global variables

// duty cycles
float duty_hs = 0;
float duty_ls = 0;

// ki, kp for hs
float ki_hs = 1.0;
float kp_hs = 1.0;
float ki_term_hs = 0;
float kp_term_hs = 0;

// ki, kp for ls
float ki_ls = 0.1;
float kp_ls = 0;
float ki_term_ls = 0;
float kp_term_ls = 0;

// current reading and reference
float I_curr = 0;
float I_ref = 0;


void setup() {
  Serial.begin(9600);

  
  // initiate input and ouput pins
  // for current reading
  pinMode(A8, INPUT);
  // for duty_hs, timer 1
  pinMode(11, OUTPUT);
  // for duty_ls, timer 3
  pinMode(5, OUTPUT);
  
  // configure timers
  
  GTCCR = _BV(TSM) | _BV(PSRSYNC) | _BV(PSRASY); // halt all timers
  
  //timer 1
  // mode selection
  // mode = 10, non-inverting mode
  TCCR1A = _BV(WGM11) | _BV(COM1A1) | _BV(COM1B1) | _BV(COM1C1);
  TCCR1B = _BV(WGM13) | _BV(CS10) ;
  // top
  ICR1 = 400;
  // duty cycle
  OCR1A = 0.0 * 400;
  
  //timer 3
  // mode selection
  // mode = 10 phase correct PWM, inverting mode
  TCCR3A = _BV(WGM31) | _BV(COM3A0) | _BV(COM3B0) | _BV(COM3C0)| _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1);
  TCCR3B = _BV(WGM33) | _BV(CS30);
  // top
  ICR3 = 400;
  // duty cycle
  OCR3A = (0) * 400;

  GTCCR = 0; // release all timers
}

void takeinputs(){
  // analog read to global variable
  int count = 0;
  int count_max = 30;
  I_curr = 0;
  while(count<count_max){
    I_curr += ((float)(analogRead(A8)/1023.0*5.0 - 2.4925)*10/count_max);
    count++;
  }
}

// hystresis control
void calc_I_ref(){
  if(0.8<I_curr<1.2){
    I_ref = I_curr;
  }
  else{
    I_ref = 1;
  }
}

void check_over_curr(){
  if(abs(I_curr)>4){
    OCR1A = 0;
    OCR3A = 400;
    delay(10);
    pinMode(11, INPUT_PULLUP);
    pinMode(5, INPUT_PULLUP);
  }
}

  
void loop() {
  Serial.println(I_curr);
  Serial.println(duty_hs);
  Serial.println(duty_ls);
  Serial.println();
 
  takeinputs();
  //check_over_curr();
  calc_I_ref();
  // calculate duty for hs
  float e_hs = I_ref - I_curr;
  ki_term_hs += ki_hs * e_hs;
  kp_term_hs = kp_hs * e_hs;
  if(ki_term_hs > 2) {
    ki_term_hs = 2;
  }
  else if(ki_term_hs < -1) {
    ki_term_hs = -1;
  }
  duty_hs = ki_term_hs + kp_term_hs;
  
  if(duty_hs > 0.7) {
    duty_hs = 0.7;
  }
  
  // calculate duty for ls
  float e_ls = (1-duty_hs) - (duty_ls+0.02);
  ki_term_ls += ki_ls * e_ls;
  kp_term_ls = kp_ls * e_ls;
  if(ki_term_ls > 2) {
    ki_term_ls = 2;
  }
  else if(ki_term_ls < -1) {
    ki_term_ls = -1;
  }
  duty_ls = ki_term_ls + kp_term_ls;
  
  // limit duty cycles
  
  if(duty_ls > 0.9) {
    duty_ls = 0.9;
  }

  // ensure dead band
  if(duty_ls+0.02 > 1-duty_hs){
    duty_ls = 1-duty_hs - 0.02;
  }
  
  // ensure dead time while updating timers
  OCR1A = duty_hs * 400;
  OCR3A = (1-duty_ls)*400;

}
