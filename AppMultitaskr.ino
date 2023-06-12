#include <SoftwareSerial.h>
#include <Servo.h>
Servo servo1; 
Servo servo2;
int accelero;
int bluetoothTx =1; //Tx to 10pin
int bluetoothRx = 0; //Rx to 11pin

SoftwareSerial bluetooth (bluetoothTx, bluetoothRx);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.setTimeout(5);
  servo1.attach(3);
  servo2.attach(5);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()>0){
    accelero = Serial.parseInt();
    if (accelero >100 && accelero <280){
      digitalWrite(10, LOW);
      delay(1);
      accelero = map(accelero, 100, 280, 0,180);
      servo1.write(accelero);
    }
    if (accelero >300 && accelero <480){
      digitalWrite(9,LOW);
      delay(1);
      accelero = map(accelero, 300, 480, 0,360);
      servo2.write(accelero);
    }
  }
}
