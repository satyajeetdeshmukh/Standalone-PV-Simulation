// Take 3 inputs and initiate pwm
// Use P&0 to calculate new duty cycle

float pv_boost_pwm = 50;
float pv_boost_pwm_delta = 1.5;
float pv_power_old = 0;
float pv_boost_pwm_inc = -1;

float pv_voltage = 0;
float pv_current = 0;
float pv_power =  0;

void setup()
{

 //Serial.begin(9600);

 pinMode(A7, INPUT); // voltage of PV system
 pinMode(A8, INPUT); // current of PV system
 pinMode(A0, INPUT); // voltage of DC-Link
 
 pinMode(9, OUTPUT);  // pin for boost pwm
 pinMode(2, OUTPUT);  // bidi - buck
 pinMode(6, OUTPUT);  // bidi - boost

 TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20) ; // non-Inverting PWM,
 OCR2A = 99;
  // Fast PWM and NO prescaling 16M/1024 // your CS20/CS21 settings are a prescaler of 32.
 TCCR2B =  _BV(WGM22) | _BV(CS21);  // Fast PWM and NO prescaling 16M/1024
 OCR2B = pv_boost_pwm;

 
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






void take_inputs(){
  pv_voltage = 0;
  pv_current = 0;
  int count = 0;
  int count_max = 1000;
  while(count<count_max){
    pv_voltage += (float)analogRead(A7)/1023.0*5.0*13.5/count_max;
    pv_current += abs((float)(analogRead(A8)/1023.0*5.0 - 2.4925)*10/count_max);
    count++;
  } 
  //Serial.println(pv_current);
  //Serial.println(pv_voltage);
  //Serial.println();
}

void loop() {

  //Input
  take_inputs();
  

  // Output
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

  if(pv_boost_pwm>75){
    pv_boost_pwm = 75;
    //pv_boost_pwm_inc = -pv_boost_pwm_inc;
  }

  if(pv_boost_pwm<40){
    pv_boost_pwm = 40;
    //pv_boost_pwm_inc = -pv_boost_pwm_inc;
  }

  
  // Update
  pv_power_old = pv_power;
  OCR2B = pv_boost_pwm;
}
  
