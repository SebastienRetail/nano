/*
digital pins 2 for ping and 3 for echo
arduino 5v
ground
*/

/*
Introduction:
Infrared obstacle avoidance sensor is equipped with distance adjustment function and is especially designed for wheeled robots. 
This sensor has strong adaptability to ambient light and is of high precision. It has a pair of infrared transmitting and receiving tube. 
When infrared ray launched by the transmitting tube encounters an obstacle (its reflector), 
the infrared ray is reflected to the receiving tube, and the indicator will light up; 
the signal output interface outputs digital signal. 
We can adjust the detection distance through the potentiometer knob ( effective distance: 2～40cm, working Voltage: 3.3V-5V ). 
Thanks to a wide voltage range, this sensor can work steadily even under fluctuating power supply voltage 
and is suitable for the use of various micro-controllers, Arduino controllers and BS2 controllers. 
A robot mounted with the sensor can sense changes in the environment.

Specification:
Working voltage: DC 3.3V-5V
Working current: ≥20mA
Working temperature: －10℃—＋50℃
Detection distance: 2-40cm
IO Interface: 4 wire interface (-/+/S/EN)
Output signal: TTL voltage
Accommodation mode: Multi-circle resistance regulation
Effective Angle: 35°
Size: 41.7*16.7mm
Weight: 5g
*/


/* LN 298 MOTOR DRIVER
 *  https://www.instructables.com/How-to-use-the-L298-Motor-Driver-Module-Arduino-Tu/
 * The connections are pretty easy!

Module 5V (or Vcc) - Arduino 5V pin
Module GND - Arduino GND pin
Module 12V (or Vbat) - To external power source up to 35V.
For this tutorial just connect it with Arduino Vin pin.
Module output 1 & 2 - Connect dc motor A
Module output 3 & 4 - Connect dc motor B
Module IN1 - Arduino pin 5
Module IN2 - Arduino pin 6
Module IN3 - Arduino pin 10
Module IN4 - Arduino pin 9
 * 
 */

#include <Servo.h>
Servo servoMotor;

//MILLIS
unsigned long interval=1000;            // the time we need to wait
unsigned long previousMillis=0;         // millis() returns an unsigned long.

 const int pingPin = 2;
const int echoPin = 3;// Echo Pin of Ultrasonic Sensor
 

const int sensorPin = 12;     // the number of the sensor pin
int sensorState = 0;         // variable for reading the sensor status


//L293D
//Motor A
const int motorPin1  = 9;  // Pin 14 of L293 input 1
const int motorPin2  = 10;  // Pin 10 of L293 input 2
//Motor B
const int motorPin3  = 6; // Pin  7 of L293 bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
const int motorPin4  = 5;  // Pin  2 of L293

const int ledPin =  13;      // the number of the LED pin
int ledpin = 13;                          //LED_BUILTIN but 13 is also the pindirection of motor B ! ? 
int RedLed = 13;
int GreenLed = 13;

//BUZZER
const int buzzerPin = 2;                  // star wars
int counter = 2;                          // star wars used to make leds blink
//Notes used to play music on buzzer 
const int c = 261;
const int d = 294;
const int e = 329;
const int f = 349;
const int g = 391;
const int gS = 415;
const int a = 440;
const int aS = 455;
const int b = 466;
const int cH = 523;
const int cSH = 554;
const int dH = 587;
const int dSH = 622;
const int eH = 659;
const int fH = 698;
const int fSH = 740;
const int gH = 784;
const int gSH = 830;
const int aH = 880;




void setup() {
  pinMode(ledPin, OUTPUT);   
  pinMode(GreenLed, OUTPUT);
  pinMode(RedLed, OUTPUT);
  
  //buzzer
  pinMode(buzzerPin, OUTPUT);
     
  pinMode(sensorPin, INPUT); 
  
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // declare the servo is on pin 11
  servoMotor.attach(11);

   //Set pins as outputs
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(motorPin3, OUTPUT);
    pinMode(motorPin4, OUTPUT);
  
  Serial.begin(9600); // Starting Serial Terminal
}

void loop(){
//  ULTRASONIC();
//  IR_AVOID();
//  servo_0();
//  servo_90();
//  servo_180();
//  servo_sweep();

//  //we use the millis to see when the last check occured, and when needed, perform another check
//    unsigned long currentMillis = millis(); // grab current time
//if ((unsigned long)(currentMillis - previousMillis) >= interval) {
//  
//    ultrasonic();
//    // irdetection(); // commented because make it simple
//    
//  // save the "current" time
//  previousMillis = millis();
//}
// delay(2); 
//  

//servo_sweep();
  
}
 


///////////////////////////////////Ultrasonic///////////////////////////////////////////
void ultrasonic(){
  long duration, inches, cm;
  // long microsecondsToInches(long microseconds) {
  // return microseconds / 74 / 2;
  // }

  //pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pingPin, LOW);
  //pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  // inches = microsecondsToInches(duration);
  cm = microsecondsToCentimeters(duration);
  if (cm < 10) {
    escape();
  }
  else motorforward();
  
  // Serial.print(inches);
  // Serial.print(" in, ");
  // Serial.print(cm);
  // Serial.print(" cm");
  // Serial.println();
  // delay(100);

}void ULTRASONIC(){
   long duration, inches, cm;
   
   digitalWrite(pingPin, LOW);
   delayMicroseconds(2);
   digitalWrite(pingPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(pingPin, LOW);
   
   duration = pulseIn(echoPin, HIGH);
   //inches = microsecondsToInches(duration);
   cm = microsecondsToCentimeters(duration);
   //Serial.print(inches);
   //Serial.print("in, ");
   Serial.print(cm);
   Serial.print("cm");
   Serial.println();
   delay(100);
}
 
long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}
 
long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}

///////////////////////////////////Ultrasonic///////////////////////////////////////////

void IR_AVOID(){
  // read the state of the sensor value:
  sensorState = digitalRead(sensorPin);
  // if it is, the sensorState is LOW:
  if (sensorState == LOW) {     
     digitalWrite(ledPin, HIGH);  
     Serial.print(sensorState);
  } 
  else {
       digitalWrite(ledPin, LOW); 
       Serial.print(sensorState);
  }
}

void servo_0() {
  
  // Desplazamos a la posición 0º
  servoMotor.write(0);
  // Esperamos 1 segundo
  delay(1000);
}
void servo_90() { 
  // Desplazamos a la posición 90º
  servoMotor.write(90);
  // Esperamos 1 segundo
  delay(1000);
} 
void servo_180() { 
  // Desplazamos a la posición 180º
  servoMotor.write(180);
  // Esperamos 1 segundo
  delay(1000);
}

void servo_sweep() {
 
  // Vamos a tener dos bucles uno para mover en sentido positivo y otro en sentido negativo
  // Para el sentido positivo
  for (int i = 0; i <= 180; i++)
  {
    // Desplazamos al ángulo correspondiente
    servoMotor.write(i);
    // Hacemos una pausa de 25ms
    delay(25);
  }
 
  // Para el sentido negativo
  for (int i = 179; i > 0; i--)
  {
    // Desplazamos al ángulo correspondiente
    servoMotor.write(i);
    // Hacemos una pausa de 25ms
    delay(25);
  }
}



void StarWars()
{
 
  //Play first section
  firstSection();
 
  //Play second section
  secondSection();
  
  //Play first Variant
  firstVariant();
 
  //Repeat second section
  secondSection();
 
  // Play second Variant
  secondVariant();

}
 
void beep(int note, int duration)
{
  //Play tone on buzzerPin
  tone(buzzerPin, note, duration);
 
  //Play different LED depending on value of 'counter'
  if(counter % 2 == 0)
  {
    digitalWrite(RedLed, HIGH);
    delay(duration);
    digitalWrite(RedLed, LOW);
  }else
  {
    digitalWrite(GreenLed, HIGH);
    delay(duration);
    digitalWrite(GreenLed, LOW);
  }
 
  //Stop tone on buzzerPin
  noTone(buzzerPin);
 
  delay(50);
 
  //Increment counter
  counter++;
}
 
void firstSection()
{
  beep(a, 500);
  beep(a, 500);    
  beep(a, 500);
  beep(f, 350);
  beep(cH, 150);  
  beep(a, 500);
  beep(f, 350);
  beep(cH, 150);
  beep(a, 650);
 
  delay(500);
 
  beep(eH, 500);
  beep(eH, 500);
  beep(eH, 500);  
  beep(fH, 350);
  beep(cH, 150);
  beep(gS, 500);
  beep(f, 350);
  beep(cH, 150);
  beep(a, 650);
 
  delay(500);
}
 
void secondSection()
{
  beep(aH, 500);
  beep(a, 300);
  beep(a, 150);
  beep(aH, 500);
  beep(gSH, 325);
  beep(gH, 175);
  beep(fSH, 125);
  beep(fH, 125);    
  beep(fSH, 250);
 
  delay(325);
 
  beep(aS, 250);
  beep(dSH, 500);
  beep(dH, 325);  
  beep(cSH, 175);  
  beep(cH, 125);  
  beep(b, 125);  
  beep(cH, 250);  
 
  delay(350);
}

void firstVariant()
{  
  beep(f, 250);  
  beep(gS, 500);  
  beep(f, 350);  
  beep(a, 125);
  beep(cH, 500);
  beep(a, 375);  
  beep(cH, 125);
  beep(eH, 650);
 
  delay(500);
  }
  
void secondVariant()
{
  beep(f, 250);  
  beep(gS, 500);  
  beep(f, 375);  
  beep(cH, 125);
  beep(a, 500);  
  beep(f, 375);  
  beep(cH, 125);
  beep(a, 650);  
 
  delay(650);  
  }


//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////MOTOR FUNCTIONS////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////Movement////////////////////////////////////////////
void motorstop() {
  digitalWrite(9, HIGH);  //Engage the Brake for Channel A
  digitalWrite(8, HIGH);  //Engage the Brake for Channel B
}

void motorforward() {
  //Motor A forward @ full speed
  digitalWrite(12, HIGH);  //Establishes backward direction of Channel A
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  analogWrite(3, 255);    //Spins the motor on Channel A at full speed

  //Motor B forward @ full speed
  digitalWrite(13, HIGH); //Establishes forward direction of Channel B
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, 255);   //Spins the motor on Channel B at full speed
}

void motorreverse() {
  //Motor A reverse @ half speed
  digitalWrite(12, LOW);  //Establishes backward direction of Channel A
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  analogWrite(3, 125);    //Spins the motor on Channel A at half speed

  //Motor B reverse @ half speed
  digitalWrite(13, LOW); //Establishes forward direction of Channel B
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, 125);   //Spins the motor on Channel B at half speed
}

void motorright() {
  //Motor A reverse @ half speed
  digitalWrite(12, LOW);  //Establishes backward direction of Channel A
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  analogWrite(3, 125);    //Spins the motor on Channel A at half speed

  //Motor B forward @ full speed
  digitalWrite(13, HIGH); //Establishes forward direction of Channel B
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, 255);   //Spins the motor on Channel B at full speed
}

void motorleft() {
  //Motor A forward @ full speed
  digitalWrite(12, HIGH);  //Establishes backward direction of Channel A
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  analogWrite(3, 255);    //Spins the motor on Channel A at full speed

  //Motor B reverse @ half speed
  digitalWrite(13, LOW); //Establishes forward direction of Channel B
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, 125);   //Spins the motor on Channel B at half speed
}

void rotateccw() {
  //Motor A reverse @ half speed
  digitalWrite(12, HIGH);  //Establishes backward direction of Channel A
  digitalWrite(9, LOW);   //Disengage the Brake for Channel A
  analogWrite(3, 255);    //Spins the motor on Channel A at full speed

  //Motor B reverse @ half speed
  digitalWrite(13, LOW); //Establishes forward direction of Channel B
  digitalWrite(8, LOW);   //Disengage the Brake for Channel B
  analogWrite(11, 255);   //Spins the motor on Channel B at full speed
}

void escape(){
  // motorstop(); // stop the motors
  // delay(2); // wait 1 second
  // motorreverse(); // reverse the motors for 1 second
  // delay(400);
  motorstop(); // stop the motors
  delay(2); // wait 1 second
  rotateccw(); // rotate for 2 seconds
  delay(900);
  motorstop(); // stop the motors
  delay(2);
  // ImperialMarchSecondVariant();
  // delay(2);
}

void motortest(){
  motorstop(); // stop the motors
  delay(2);
  motorforward();
  delay(1000);
  motorstop(); // stop the motors
  delay(2);
  motorright();
  delay(1000);
  motorstop(); // stop the motors
  delay(2);
  motorleft();
  delay(1000);
  motorstop(); // stop the motors
  delay(2);
  motorreverse(); // reverse the motors for 1 second
  delay(1000);
  motorstop(); // stop the motors
  delay(2); // wait 1 second
  rotateccw(); // rotate for 2 seconds
  delay(900);
  motorstop(); // stop the motors
  delay(2);
}
//////////////////////////////////Movement////////////////////////////////////////////

void motor_test(){
 
    //Set pins as outputs
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(motorPin3, OUTPUT);
    pinMode(motorPin4, OUTPUT);
    
    //Motor Control - Motor A: motorPin1,motorpin2 & Motor B: motorpin3,motorpin4

    //This code  will turn Motor A clockwise for 2 sec.
    analogWrite(motorPin1, 180);
    analogWrite(motorPin2, 0);
    analogWrite(motorPin3, 180);
    analogWrite(motorPin4, 0);
    delay(5000); 
    //This code will turn Motor A counter-clockwise for 2 sec.
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 180);
    analogWrite(motorPin3, 0);
    analogWrite(motorPin4, 180);
    delay(5000);
    
    //This code will turn Motor B clockwise for 2 sec.
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 180);
    analogWrite(motorPin3, 180);
    analogWrite(motorPin4, 0);
    delay(1000); 
    //This code will turn Motor B counter-clockwise for 2 sec.
    analogWrite(motorPin1, 180);
    analogWrite(motorPin2, 0);
    analogWrite(motorPin3, 0);
    analogWrite(motorPin4, 180);
    delay(1000);    
    
    //And this code will stop motors
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, 0);
    analogWrite(motorPin3, 0);
    analogWrite(motorPin4, 0);
  
}
