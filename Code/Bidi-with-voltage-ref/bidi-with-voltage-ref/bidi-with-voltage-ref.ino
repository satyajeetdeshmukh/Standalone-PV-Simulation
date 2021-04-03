// global variables

// duty cycles
double duty_hs = 0;
double duty_ls = 0;

// duty cycle parameters
double duty_ls_delta = 0.0005;
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


// current reading and reference
double Ib_read = 0;
double Ib_ref = 0;
double Vdc_read = 0;
double Vdc_ref = 20;

// soft start vars
int soft_start_mode = 1;
int relay_on = 0;

// Pins
const int pin_Ib = A8;// Ib current pin
const int pin_Vdc = A2;// Vdc-link voltage pin
const int pin_duty_hs = 11; // duty_hs pin
const int pin_duty_ls = 5;// duty_ls pin
const int pin_vcc_ib = 34;// Vcc for Ib sensor
const int pin_relay_bat = 46;// Battery relay pin

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

  // initialize pins
  digitalWrite(pin_vcc_ib, HIGH); // for ib sensor vcc
  digitalWrite(pin_relay_bat, LOW); // for relay

  // configure timers
  configure_timers();
}

void takeinputs() {
  // analog read to global variable
  int count = 0;
  int count_max = 30;
  Ib_read = 0;
  Vdc_read = 0;
  while (count < count_max) {
    Ib_read += ((double)(analogRead(pin_Ib) / 1023.0 * 5.0 - 2.4925 + 0.155) * 10 / count_max);
    Vdc_read += (double)(analogRead(pin_Vdc) / 1023.0 * 5.0 * 11 / count_max);
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
  if(abs(e_vf/Vdc_ref)<0.025){
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

void serial_print() {
  //Serial.println(duty_hs);
  //Serial.println(duty_ls);
  //Serial.println(relay_on);
  Serial.println(Vdc_read);
  //Serial.println(Ib_read);
  //Serial.println(Ib_ref);
  Serial.println();
}

void soft_start() {
  if (millis() > 5000 && relay_on == 0) {
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
  if (millis() > 7000) {
    ki_hs = post_soft_ki;
    kp_hs = post_soft_kp;
    // now controller will be in post soft start mode.
    soft_start_mode = 0;
  }
}

void loop() {

  serial_print();

  // Battery relay is started 5 secs after controller starts. clear all initial kp, ki terms. after 2 seconds, change to post soft start mode.
  if (soft_start_mode == 1) {
    soft_start();
  }

  takeinputs();

  if (millis() > 5000) {
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
