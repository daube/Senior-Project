#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified backpack

int val = 0;
int achan = 0;
int bchan = 0;


void setup() {
  pinMode(5, OUTPUT);  //PWM signal output
  pinMode(A0, INPUT);  //Input from POT to change PWM signal
  pinMode(2, INPUT);  //Input from Encoder Channel A
  pinMode(3, INPUT);  //Input from Encoder Channel B
  pinMode(8, OUTPUT); //Output to disable pin on Gate driver
  TCCR0A = 0;
  TCCR0B = 0;
  TCCR0A |= (1 << WGM00) | (1 << COM0A1);
  TCCR0B |= (1 << CS00);
  Serial.begin(9600);

   // activate LCD module
  lcd.begin (16,2); // for 16 x 2 LCD module
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);
  
  
  attachInterrupt(digitalPinToInterrupt(2), count, RISING); //pin 2, goes to count function, detects rising edge
  
  cli();//stop interrupts
  
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 6249;// = (16*10^6) / (256*10) - 1 (must be <65536), interupt every 100 milisecond
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10, CS11, and CS12 bits for 256 prescaler
  TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts
}

double counter = 0; 
double RPM = 0; 

void loop() {
  val = analogRead(A0);  //Divide by 4 because analogRead is 1024 but write is 256
  achan = digitalRead(2); //read Channel A of Encoder
  bchan = digitalRead(3); //read Channel B of Encoder 
  analogWrite(5, (val/4)); //PWM signal (pin#, DutyCycle), Divide by 4 because analogRead is 1024 but write is 256
  digitalWrite(8, LOW); //Output 0 to pin 8, when pin low, disable on gatedriver is not disabling the circuit, when disable is high it does

  lcd.home (); // set cursor to 0,0
  lcd.print("I am Motor"); 
  lcd.setCursor(0,1); //0 = start of lcd left side, 1 corresponds to second row
  lcd.print("RPM:");
  lcd.setCursor(4,1);
  lcd.print((int) RPM); //int to have integers
  lcd.print("      ");  //clearing

  }



 ISR(TIMER1_COMPA_vect){
   RPM = (counter/1440)*600;
   Serial.println(counter);
   counter = 0;
  }


 void count(){
  counter++;
 }
 

