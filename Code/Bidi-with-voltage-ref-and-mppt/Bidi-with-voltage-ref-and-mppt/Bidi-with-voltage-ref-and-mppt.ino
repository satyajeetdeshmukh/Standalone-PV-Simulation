// global variables

// duty cycles
double duty_hs = 0;
double duty_ls = 0;

double duty_ls_delta = 0.0005;

double dead_band = 0.05;

double kp_vf=0.01;
double ki_vf=0.0002;

double ki_term_vf=0;
double kp_term_vf=0;



// ki, kp for hs
double post_soft_kp = 0.0001;
double post_soft_ki = 0.0001;


double kp_hs = 0.0001;
double ki_hs = 0.0001;

double ki_term_hs = 0;
double kp_term_hs = 0;


// current reading and reference
double Ib_read = 0;
double I_ref = 0;
double Vdc_read = 0;
double Vdc_ref = 20;

// mppt
double pv_boost_pwm = 50;
double pv_boost_pwm_delta = 0.25;
double pv_power_old = 0;
double pv_boost_pwm_inc = -1;
double pv_voltage = 0;
double pv_current = 0;
double pv_power =  0;


void setup() {
  Serial.begin(9600);
  // initiate input and ouput pins
 
  pinMode(A8, INPUT); // for battery current reading
  pinMode(A2, INPUT); // for Vdc-link
  pinMode(A7, INPUT); // voltage of PV system
  pinMode(A5, INPUT); // current of PV system
  
  
  pinMode(11, OUTPUT); // for duty_hs, timer 1
  pinMode(5, OUTPUT);  // for duty_ls, timer 3
  pinMode(9, OUTPUT);  // pin for boost pwm, timer 2

  pinMode(34, OUTPUT);  // Vcc for battery current sensor
  pinMode(49, OUTPUT);  // signal for relay
  pinMode(47, OUTPUT);  // Vcc for relay

  digitalWrite(49, HIGH); // initialize low
  
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

  // timer 2
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20) ; // non-Inverting PWM,
  OCR2A = 99;
  // Fast PWM and NO prescaling 16M/1024 // your CS20/CS21 settings are a prescaler of 32.
  TCCR2B =  _BV(WGM22) | _BV(CS21);  // Fast PWM and NO prescaling 16M/1024
  OCR2B = pv_boost_pwm;
  
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
  int count_max = 100;
  Ib_read = 0;
  Vdc_read = 0;
  pv_voltage = 0;
  pv_current = 0;
  while(count<count_max){
    Ib_read += ((double)(analogRead(A8)/1023.0*5.0 - 2.4925 +0.15)*10/count_max);
    Vdc_read += (double)(analogRead(A2)/1023.0*5.0*11*20/19.15/count_max);
    pv_voltage += (float)analogRead(A7)/1023.0*5.0*13.5*10/11.9/count_max;
    pv_current += abs((double)(analogRead(A5)/1023.0*5.0- 2.497)*10/count_max);
    count++;
  }
}

void check_over_curr(){
  if(abs(Ib_read)>4){
    OCR1A = 0;
    OCR3A = 400;
    delay(10);
    pinMode(11, INPUT_PULLUP);
    pinMode(5, INPUT_PULLUP);
  }
}

void mppt(){
  pv_power =  pv_voltage * pv_current;
  if(pv_power<pv_power_old)
  {
    pv_boost_pwm_inc = -pv_boost_pwm_inc;
    //DeltaD=DeltaD_Max/100;
//    if(pv_power-pv_power_old > pv_power*0.1 || pv_power_old-pv_power > pv_power*0.1){
//      pv_boost_pwm_delta=0.01;
//    }
  }
  pv_boost_pwm = pv_boost_pwm + pv_boost_pwm_inc * pv_boost_pwm_delta;

  if(pv_boost_pwm>80){
    pv_boost_pwm = 80;
  }
  else if(pv_boost_pwm<20){
    pv_boost_pwm = 20;
  }
  // Update
  pv_power_old = pv_power;
  OCR2B = pv_boost_pwm;
}

void soft_start(){
  if(abs(Ib_read)>0.2){
    ki_hs = post_soft_ki;
    kp_hs = post_soft_kp;
    if(ki_term_vf > 4) {
      ki_term_vf = 4;
    }
    else if(ki_term_vf < -4) {
      ki_term_vf = -4;
    }
  }
  else {
    kp_term_vf = 0;
    if(duty_hs > 0.05){
      duty_hs = 0.05;
    }
    if(duty_ls > 0.05){
      duty_ls = 0.05;
    }
    if(ki_term_vf > 0.2) {
      ki_term_vf = 0.2;
    }
    else if(ki_term_vf < -0.2) {
      ki_term_vf = -0.2;
    }
  }
}
void loop() {
  Serial.println(duty_hs);
  Serial.println(duty_ls);
  Serial.println(Vdc_read);
  Serial.println(Ib_read);
  Serial.println(I_ref);
  Serial.println();

  
  takeinputs();
  mppt();

  // calc i_ref from voltage ref
  double e_vf = Vdc_read - Vdc_ref;
  ki_term_vf += ki_vf * e_vf;
  kp_term_vf = kp_vf * e_vf;
  if(ki_term_vf > 4) {
    ki_term_vf = 4;
  }
  else if(ki_term_vf < -4) {
    ki_term_vf = -4;
  }
  soft_start();
  I_ref = ki_term_vf + kp_term_vf;
  
  
  
  // calculate duty for hs
  double e_hs = I_ref - Ib_read;
  ki_term_hs += ki_hs * e_hs;
  kp_term_hs = kp_hs * e_hs;
  if(ki_term_hs > 1.5) {
    ki_term_hs = 1.5;
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

  digitalWrite(47, HIGH); // vcc of relay
  if(millis()>5*1000){
    digitalWrite(49, LOW);
  }
  else {
    duty_ls=0;
    duty_hs=0;
  }
  digitalWrite(34, HIGH); // Vcc for battery current sensor
  
  //Update timers
  OCR1A = duty_hs * 400;
  OCR3A = (1-duty_ls)*400;
  
  
}
