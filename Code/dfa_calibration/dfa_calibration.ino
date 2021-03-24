double DAF1,DAF2,DAF3;

void setup() {
   pinMode(A7, INPUT);
   pinMode(A8, INPUT); // current of PV system
   pinMode(A0, INPUT);
  // put your setup code here, to run once:
  Serial.begin(9600);
}
void take_inputs(){
  DAF1=0;
  DAF2=0;
  DAF3=0;
  
  int count = 0;
  int count_max = 30;
  while(count<count_max){
    DAF1 += (float)analogRead(A7)/1023.0*5.0*13.5*10/11.9/count_max;
    DAF2 += (float)analogRead(A8)/1023.0*5.0*13.5*30/34/count_max;
    DAF3 += (float)analogRead(A0)/1023.0*5.0*13.5/count_max;
    count++;
  } 
  Serial.println(DAF1);
  Serial.println(DAF2);
  Serial.println(DAF3);
  Serial.println();
}

void loop() { 
    take_inputs();
}
