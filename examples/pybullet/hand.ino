//arduino script for vr glove, sending analogue 'finger' readings
//to be used with pybullet and hand.py


int sensorPin0 = A0;
int sensorPin1 = A1;
int sensorPin2 = A2;
int sensorPin3 = A3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  int sensorValue0 = analogRead(sensorPin0);
  int sensorValue1 = analogRead(sensorPin1);
  int sensorValue2 = analogRead(sensorPin2);
  int sensorValue3 = analogRead(sensorPin3);

  Serial.print(",");
  Serial.print(sensorValue0);
  Serial.print(",");
  Serial.print(sensorValue1);
  Serial.print(",");
  Serial.print(sensorValue2);
  Serial.print(",");
  Serial.print(sensorValue3);  
  Serial.println(",");
  delay(10);
}
