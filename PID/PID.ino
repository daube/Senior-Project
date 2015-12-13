#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>


LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified backpack

int desired = 0; //used to get PWM duty cycle
int achan = 0;  // ChannelA of the encoder - Actual speed
//int bchan = 0;
int Error = 0;
int PIDspeed = 0;
int OutputSignal = 0;
int PrevSpeed = 0;
int Integral = 0;
int P = 0;
int I = 0;
int D = 0;


void setup() {
  pinMode(5, OUTPUT);  //PWM signal output
  pinMode(A0, INPUT);  //Input from POT to change PWM signal
  pinMode(2, INPUT);  //Input from Encoder Channel A
  pinMode(3, INPUT);  //Input from Encoder Channel B
  pinMode(8, OUTPUT); //Output to disable pin on Gate driver
  
  //setting the frequency to 31.25kHz by not setting a prescaler to Timer0
  TCCR0A = 0;
  TCCR0B = 0;
  TCCR0A |= (1 << WGM00) | (1 << COM0A1); // a|=b > a= a|b bitwise OR
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
  // set compare match register for 1khz increments
  OCR1A = 6249;// = (16*10^6) / (256*100) - 1 (must be <65536), interupt every 100 milisecond
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10, CS11, and CS12 bits for 256 prescaler
  TCCR1B |= (1 << CS12) | (0 << CS11) | (0 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

//  TCCR2A = 0;// set entire TCCR2A register to 0
//  TCCR2B = 0;// same for TCCR2B
//  TCNT2  = 0;//initialize counter value to 0
//  // set compare match register for 1hz increments
//  OCR2A = 124;// = (16*10^6) / (64*124) - 1 (must be <255), interupt every 1 milisecond
//  // turn on CTC mode
//  TCCR2B |= (1 << WGM22);
//  // Set CS20, CS21, and CS22 bits for 64 prescaler
//  TCCR2B |= (1 << CS22) | (0 << CS21) | (0 << CS20);   
//  // enable timer compare interrupt
//  TIMSK2 |= (1 << OCIE2A);

  sei();//allow interrupts

}

double counter = 0; 
double RPM = 0; 
float kP = .01; //proportional constant
float kI = 0; //integral constant
float kD = 0; //derivative constant

void loop() {
  desired = analogRead(A0); //desired speed of motor as judged by the POT, 1024 number
  achan = digitalRead(2); //read Channel A of Encoder, actual speed of the motor in frequency
  //bchan = digitalRead(3); //read Channel B of Encoder 
  analogWrite(5, (PIDspeed/4)); //PWM signal (pin#, DutyCycle), Divide by 4 because analogRead is 1024 but write is 256
  digitalWrite(8, LOW); //Output 0 to pin 8, when pin low, disable on gatedriver is not disabling the circuit, when disable is high it does

  lcd.home (); // set cursor to 0,0
  lcd.print("Vibration Station"); 
  lcd.setCursor(0,1); //0 = start of lcd left side, 1 corresponds to second row
  lcd.print("RPM:");
  lcd.setCursor(4,1);
  lcd.print((int) RPM); //int to have integers
  lcd.print("      ");  //clearing

  }

//RPM determining function, PID as well
 ISR(TIMER1_COMPA_vect){
   RPM = (counter/1440)*600; //divide the amount of up pulses by 1440(the ppr of the encoder) and mult by 600 to get in minutes, RPM
   //Serial.println(counter); //testing purposes
   counter = 0;

   //PID Function 
   Error = desired - RPM;
   Integral = Integral + Error;
   P = abs(Error) * kP;
   I = Integral * kI;
   D = (PrevSpeed - RPM) * kD;
   PIDspeed = P + I + D; // Number should be in 0 - 1024
//   if(PIDspeed > 1000) PIDspeed = 1000; // don't want the speed of the motor either having the PWM all on
   if(PIDspeed < 0) PIDspeed = 0;  // Don't want something odd happening with a negative number
   PrevSpeed = RPM;
   Serial.print(P);
   Serial.print(",");
   Serial.print(desired);
   Serial.print(",");
   Serial.print(RPM);
   Serial.print(",");
   Serial.println(PIDspeed);
   
   
  }


 void count(){
  counter++;  //increment counter by 1 every time there is a high pulse from ChannelA of the encoder
 }
 

