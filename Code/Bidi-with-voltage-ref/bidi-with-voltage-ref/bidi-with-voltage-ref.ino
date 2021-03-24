// global variables

// duty cycles
double duty_hs = 0;
double duty_ls = 0;

double duty_ls_delta = 0.0005;

double dead_band = 0.05;

double kp_vf=0.01;
double ki_vf=0.00001;

double ki_term_vf=0;
double kp_term_vf=0;



// ki, kp for hs
double post_soft_kp = 0.05;
double post_soft_ki = 0.01;


double kp_hs = 0.0;
double ki_hs = 0.01;

double ki_term_hs = 0;
double kp_term_hs = 0;


// current reading and reference
double I_read = 0;
double I_ref = 0;
double Vdc_read = 0;
double Vdc_ref = 20;


void setup() {
  Serial.begin(9600);
  // initiate input and ouput pins
  // for current reading
  pinMode(A8, INPUT);
  // for Vdc
  pinMode(A4, INPUT);
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
  I_read = 0;
  Vdc_read = 0;
  while(count<count_max){
    I_read += ((double)(analogRead(A8)/1023.0*5.0 - 2.4925)*10/count_max);
    Vdc_read += (double)(analogRead(A4)/1023.0*5.0*13.5*30/34/count_max);
    count++;
  }
}

void check_over_curr(){
  if(abs(I_read)>4){
    OCR1A = 0;
    OCR3A = 400;
    delay(10);
    pinMode(11, INPUT_PULLUP);
    pinMode(5, INPUT_PULLUP);
  }
}

  
void loop() {
  
  Serial.println(Vdc_read);
  Serial.println(I_ref);
  Serial.println();
  
  if(millis()>50){
    ki_hs = post_soft_ki;
    kp_hs = post_soft_kp;
  }    
  
  takeinputs();

  // calc i_ref from voltage ref
  double e_vf = Vdc_read - Vdc_ref;
  ki_term_vf += ki_vf * e_vf;
  kp_term_vf = kp_vf * e_vf;
//  if(ki_term_vf > 6) {
//    ki_term_vf = 6;
//  }
//  else if(ki_term_vf < -6) {
//    ki_term_vf = -6;
//  }
  I_ref = ki_term_vf + kp_term_vf;

  
  
  // calculate duty for hs
  double e_hs = I_ref - I_read;
  ki_term_hs += ki_hs * e_hs;
  kp_term_hs = kp_hs * e_hs;
  if(ki_term_hs > 2) {
    ki_term_hs = 2;
  }
  else if(ki_term_hs < -1) {
    ki_term_hs = -1;
  }
  duty_hs = ki_term_hs + kp_term_hs;
  
  if(duty_hs > 0.9) {
    duty_hs = 0.9;
  }
  else if(duty_hs < 0){
    duty_hs = 0;  
  }
 
  // calculate duty for ls
  
  if (duty_ls<1-(duty_hs+dead_band)){
    duty_ls=duty_ls+duty_ls_delta;
  }
  else if (duty_ls>1-(duty_hs+dead_band)){
    duty_ls=1-(duty_hs+dead_band);
  }
  
  if(0>duty_ls){
    duty_ls = 0;
  }
  else if(duty_ls>0.9){
    duty_ls = 0.9;
  }
  
  // ensure dead band
  if(duty_ls+dead_band > 1-duty_hs){
    duty_ls = 1-duty_hs - dead_band;
  }
  
  //Update timers
  OCR1A = duty_hs * 400;
  OCR3A = (1-duty_ls)*400;
  
}
