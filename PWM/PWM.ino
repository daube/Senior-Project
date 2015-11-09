int val = 0;
int achan = 0;
int bchan = 0;

void setup() {
  pinMode(3, OUTPUT);  //PWM signal output
  pinMode(A0, INPUT);  //Input from POT to change PWM signal
  pinMode(4, INPUT);  //Input from Encoder Channel A
  pinMode(5, INPUT);  //Input from Encoder Channel B
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;  //change frequency to 31.25kHz
  Serial.begin(9600);

  cli();//stop interrupts
  
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15999;// = (16*10^6) / (1*1000) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
}

void loop() {
  val = analogRead(A0);  //Divide by 4 because analogRead is 1024 but write is 256
  achan = digitalRead(4); 
  bchan = digitalRead(5);
  analogWrite(3, (val/4)); //PWM signal (pin#, DutyCycle), Divide by 4 because analogRead is 1024 but write is 256

  
  
  }

  void ISR(TIMER1_COMPA_vect){
    
  }

