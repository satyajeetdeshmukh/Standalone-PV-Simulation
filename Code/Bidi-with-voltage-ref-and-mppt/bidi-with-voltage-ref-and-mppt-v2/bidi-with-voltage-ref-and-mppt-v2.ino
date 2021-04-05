// global variables
int debug_mode = 1;
int mppt_mode = 1;
int relay_on_time = 5000;
double Vdc_ref = 34;

// duty cycles
double duty_hs = 0;
double duty_ls = 0;

// duty cycle parameters
double duty_ls_delta = 0.001;
double dead_band = 0.05;

// PI of iref generator
double kp_vf = 0.005;
double ki_vf = 0.001;
double ki_term_vf = 0;
double kp_term_vf = 0;


// ki, kp for hs
double post_soft_kp = 0.05;
double post_soft_ki = 0.01;
double ki_term_hs = 0;
double kp_term_hs = 0;

// for soft start
double kp_hs = 0.0;
double ki_hs = 0.005;




// soft start vars
int soft_start_mode = 1;
int relay_on = 0;

// Pins
const int pin_Ib = A8;// Ib current pin
const int pin_Vdc = A12;// Vdc-link voltage pin
const int pin_duty_hs = 11; // duty_hs pin
const int pin_duty_ls = 5;// duty_ls pin
const int pin_vcc_ib = 34;// Vcc for Ib sensor
const int pin_relay_bat = 46;// Battery relay pin
const int pin_Vpv = A9; // Voltage of PV
const int pin_Ipv = A5; // Current of PV
const int pin_duty_pv = 9; // pwm of pv

// current reading and reference
double Ib_read = 0;
double Ib_ref = 0;
double Vdc_read = 0;

double Vpv_read = 0;
double Ipv_read = 0;

void configure_timers() {
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
  TCCR3A = _BV(WGM31) | _BV(COM3A0) | _BV(COM3B0) | _BV(COM3C0) | _BV(COM3A1) | _BV(COM3B1) | _BV(COM3C1);
  TCCR3B = _BV(WGM33) | _BV(CS30);
  // top
  ICR3 = 400;
  // duty cycle
  OCR3A = (0) * 400;


  // timer 2 for mppt
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20) ; // non-Inverting PWM,
  OCR2A = 99;
  // Fast PWM and NO prescaling 16M/1024 // your CS20/CS21 settings are a prescaler of 32.
  TCCR2B =  _BV(WGM22) | _BV(CS21);  // Fast PWM and NO prescaling 16M/1024
  OCR2B = 0 * 100;

  GTCCR = 0; // release all timers
}

void setup() {
  Serial.begin(9600);

  // declare pins
  pinMode(pin_Ib, INPUT); // for current reading
  pinMode(pin_Vdc, INPUT); // for Vdc
  pinMode(pin_duty_hs, OUTPUT); // for duty_hs, timer 1
  pinMode(pin_duty_ls, OUTPUT); // for duty_ls, timer 3
  pinMode(pin_vcc_ib, OUTPUT); // Ib current sensor Vcc
  pinMode(pin_relay_bat, OUTPUT); // relay for battery
  pinMode(pin_duty_pv, OUTPUT); // duty of boost pv

  // initialize pins
  digitalWrite(pin_vcc_ib, HIGH); // for ib sensor vcc
  digitalWrite(pin_relay_bat, LOW); // for relay

  // configure timers
  configure_timers();
}

void takeinputs() {
  // analog read to global variable
  int count = 0;
  int count_max_i = 30;
  int count_max_v = 100;
  Ib_read = 0;
  Vdc_read = 0;
  Vpv_read = 0;
  Ipv_read = 0;
  while (count < count_max_i) {
    Ib_read += ((double)(analogRead(pin_Ib) / 1023.0 * 5.0 - 2.4925 + 0.155) * 10 / count_max_i);
    Ipv_read += abs((double)(analogRead(pin_Ipv) / 1023.0 * 5.0 - 2.4925 - 0.01) * 10 / count_max_i);
    count++;
  }
  count = 0;
  while (count < count_max_v) {
    Vdc_read += (double)(analogRead(pin_Vdc) / 1023.0 * 5.0 * 11 / count_max_v);
    Vpv_read += (double)(analogRead(pin_Vpv) / 1023.0 * 5.0 * 11 / count_max_v);
    count++;
  }
}

void check_over_curr() {
  if (abs(Ib_read) > 4) {
    OCR1A = 0;
    OCR3A = 400;
    delay(10);
    pinMode(pin_duty_hs, INPUT_PULLUP);
    pinMode(pin_duty_ls, INPUT_PULLUP);
  }
}

void update_timers() {
  OCR1A = duty_hs * 400;
  OCR3A = (1 - duty_ls) * 400;
}

void calc_iref() {
  double e_vf = Vdc_read - Vdc_ref;
  if (abs(e_vf / Vdc_ref) < 0.025) {
    e_vf = 0;
  }
  ki_term_vf += ki_vf * e_vf;
  kp_term_vf = kp_vf * e_vf;
  if (ki_term_vf > 3.5) {
    ki_term_vf = 3.5;
  }
  else if (ki_term_vf < -3.5) {
    ki_term_vf = -3.5;
  }
  Ib_ref = ki_term_vf + kp_term_vf;
}

void ensure_dead_band() {
  if (duty_ls + dead_band > 1 - duty_hs) {
    duty_ls = 1 - duty_hs - dead_band;
  }
}

void calc_and_limit_duty_ls() {

  if (duty_ls < 1 - (duty_hs + dead_band)) {
    duty_ls = duty_ls + duty_ls_delta;
  }
  else if (duty_ls > 1 - (duty_hs + dead_band)) {
    duty_ls = 1 - (duty_hs + dead_band);
  }

  if (0 > duty_ls) {
    duty_ls = 0;
  }
  else if (duty_ls > 0.9) {
    duty_ls = 0.9;
  }
}

void calc_and_limit_duty_hs() {

  double e_hs = Ib_ref - Ib_read;
  ki_term_hs += ki_hs * e_hs;
  kp_term_hs = kp_hs * e_hs;
  if (ki_term_hs > 1.25) {
    ki_term_hs = 1.25;
  }
  else if (ki_term_hs < -0.25) {
    ki_term_hs = -0.25;
  }
  duty_hs = ki_term_hs + kp_term_hs;

  if (duty_hs > 0.9) {
    duty_hs = 0.9;
  }
  else if (duty_hs < 0) {
    duty_hs = 0;
  }
}



void soft_start() {
  if (millis() > relay_on_time && relay_on == 0) {
    digitalWrite(pin_relay_bat, HIGH); // turn on battery relay
    relay_on = 1;

    // clear PI
    duty_hs = 0;
    duty_ls = 0;
    ki_term_hs = 0;
    kp_term_hs = 0;
    ki_term_vf = 0;
    kp_term_vf = 0;
    Ib_ref = 0;

  }
  if (millis() > relay_on_time + 2000) {
    ki_hs = post_soft_ki;
    kp_hs = post_soft_kp;
    // now controller will be in post soft start mode.
    soft_start_mode = 0;
  }
}

double pv_power_old = 0;
int pv_boost_duty_inc = 1;
double pv_boost_duty = 0.5;
double pv_boost_duty_delta = 0.005;
int count_power = 0;
int max_count_power = 1;
double pv_power = 0;

void mppt_func() {

  while (count_power < max_count_power) {
    pv_power +=  (Vpv_read * Ipv_read) / max_count_power;
    count_power++;
  }

  if (count_power == max_count_power) {
    if (pv_power < pv_power_old)
    {
      pv_boost_duty_inc = -pv_boost_duty_inc;
      //DeltaD=DeltaD_Max/100;
      //    if(pv_power-pv_power_old > pv_power*0.1 || pv_power_old-pv_power > pv_power*0.1){
      //      pv_boost_duty_delta=0.01;
      //    }
    }
    pv_boost_duty = pv_boost_duty + pv_boost_duty_inc * pv_boost_duty_delta;

    if (pv_boost_duty > 0.8) {
      pv_boost_duty = 0.8;
    }
    else if (pv_boost_duty < 0.2) {
      pv_boost_duty = 0.2;
    }
    // Update
    pv_power_old = pv_power;
    OCR2B = pv_boost_duty * 100;
    count_power = 0;
    pv_power = 0;
  }
}

void serial_print() {
  //Serial.println(duty_hs);
  //Serial.println(duty_ls);
  //Serial.println(relay_on);
  Serial.println(Vdc_read);
  Serial.println(Ib_read);
  //Serial.println(Ib_ref);
  Serial.println(Vpv_read);
  Serial.println(Ipv_read);
  //Serial.println(pv_boost_duty);
  Serial.println();
}

void loop() {

  serial_print();

  // Battery relay is started 5 secs after controller starts. clear all initial kp, ki terms. after 2 seconds, change to post soft start mode.
  if (soft_start_mode == 1) {
    soft_start();
  }

  takeinputs();

  if (mppt_mode == 1) {
    mppt_func();
  }

  if (millis() > relay_on_time) {
    // calc Ib_ref from voltage ref
    calc_iref();

    // calculate duty for hs using Ib_ref
    calc_and_limit_duty_hs();

    // calculate duty for ls by following duty_hs
    calc_and_limit_duty_ls();

    ensure_dead_band();
    update_timers();
  }


}
