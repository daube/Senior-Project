int val = 0;

void setup() {
  pinMode(3, OUTPUT);  //PWM signal output
  pinMode(A0, INPUT);  //Input from POT to change PWM signal
  //pinMode(A1, INPUT);  //Input from Encoder Channel A
  //pinMode(A2, INPUT);  //Input from Encoder Channel B
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;  //change frequency to 31.25kHz
}

void loop() {
  val = analogRead(A0)/4;  //Divide by 4 because analogRead is 1024 but write is 256
  analogWrite(3, val);     //PWM signal (pin#, DutyCycle)
  
  
}
