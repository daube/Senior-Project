void setup() {
  pinMode(3,OUTPUT);
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;
}

void loop() {
  // put your main code here, to run repeatedly:
analogWrite(3,200);
}
