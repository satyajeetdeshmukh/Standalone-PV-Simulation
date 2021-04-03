void setup() {
  pinMode(A8,INPUT);
  Serial.begin(9600);
  
  // put your setup code here, to run once:

}

void loop() {
  float voltage = (float)analogRead(A8)/1023.0*5.0*11 + 1;
  Serial.println(voltage);
  // put your main code here, to run repeatedly:
}
