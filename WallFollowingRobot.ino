#include <Servo.h>

//Variables
float refDistance = 20.000;
float outDistance;
float distanceError;
int enablePinA = 5;
int enablePinB = 6;
int servoPin = 3;
int dirA1 = 7;
int dirA2 = 8;
int dirB1 = 9;
int dirB2 = 11;
float duration;
int algVA; // Analog voltage to motor set A
int algVB; // Analog voltage to motor set B
int maxV = 150;
int minV = 0;
float preError = 0;
float sumError = 0;
float sumArray[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int sample_time;
unsigned long start_time;

//PID Constants
int kp = 10;
int ki = 1;

Servo ultServo; // Instance of the Servo object.
void setup() {
  Serial.begin(9600);
  pinMode(A5, OUTPUT); // Ultrasonic sensor TRIG Pin
  pinMode(A4, INPUT); // Ultrasonic sensor ECHO Pin
  
  //Left Motor set (A)
  pinMode(enablePinA, OUTPUT); //Enable
  pinMode(dirA1, OUTPUT); // In1
  pinMode(dirA2, OUTPUT); // In2

  //Right Motor set
  pinMode(enablePinB, OUTPUT); //Enable
  pinMode(dirB1, OUTPUT); // In1
  pinMode(dirB2, OUTPUT); // In2

  //Servo pin on Arduino
  ultServo.attach(servoPin);
  
  //Set servo to 180 degrees
  ultServo.write(0);
  
  digitalWrite(dirA1, HIGH); // In1
  digitalWrite(dirA2, LOW); // In2

  digitalWrite(dirB1, LOW); // In1
  digitalWrite(dirB2, HIGH); // In2
  
  delay(2000);
  start_time = millis();
}

void loop() {

  //Calculating distance
  digitalWrite(A5, HIGH);
  delayMicroseconds(1000);
  digitalWrite(A5, LOW);
  duration = pulseIn(A4, HIGH);
  outDistance = 0.017*duration; // output distance measured by ultrasonic sensor in cm
  //Serial.println(outDistance);

  //Distance error
  distanceError = refDistance - outDistance;
  sample_time = millis()- start_time;
  start_time = millis();
  
  // Defining the elements of the array
  for(int count = 0; count < 9; count++){
    sumArray[count] = sumArray[count+1];
    }
    sumArray[9] = distanceError;
  
  sumError = 0;
  // Sum the error array to implement an integral controller
  for(int count = 0; count < 10; count++){
    sumError = sumError + sumArray[count];
    }
      
  algVA = maxV + (kp*distanceError) + (ki*sumError*sample_time/1000);
  algVB = maxV - (kp*distanceError) - (ki*sumError*sample_time/1000);  

  
  algVA = constrain(algVA, minV, maxV);
  algVB = constrain(algVB, minV, maxV);
  
  analogWrite(enablePinA, algVA); //Enable
  analogWrite(enablePinB, algVB); //Enable
  delay(300);

  Serial.print(" VA = ");
  Serial.print(algVA);
  Serial.print(" VB = ");
  Serial.print(algVB);
  Serial.print(" Error = ");
  Serial.println(distanceError);
}
