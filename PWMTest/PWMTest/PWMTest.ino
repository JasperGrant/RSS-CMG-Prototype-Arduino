#include <Servo.h>

Servo myservo;

void setup() {
  // put your setup code here, to run once:
  pinMode(19, OUTPUT);
  pinMode(20, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);

  Serial.begin(9600);

    myservo.attach(16);
  

}

void loop() {
  
  // put your main code here, to run repeatedly:
  analogWrite(20, 200);
  digitalWrite(19, LOW);
  digitalWrite(13, HIGH);
 
  Serial.println(map(analogRead(A9),0,1023,-5000,5000));
   
   myservo.write(0);
   delay(20000);
   digitalWrite(19, HIGH);
   delay(10000);
   myservo.write(45);
   delay(20000);
  

}
