#include <SimpleDHT.h>
int pinDHT11 = 2;
int pinRELAY = 8;
SimpleDHT11 dht11;
bool runFan = false;


void setup() {
  while(!Serial);
  Serial.begin(9600);
  pinMode(pinRELAY, OUTPUT);
  digitalWrite(pinRELAY, LOW);
}

void loop() {
  byte temperature = 0;
  byte humidity = 0;
  int err = SimpleDHTErrSuccess;

  err = dht11.read(pinDHT11, &temperature, &humidity, NULL);
  
  if (err == SimpleDHTErrSuccess) {
    //Serial.print("temp=");  Serial.println((int)temperature);
    //Serial.print("humi=");  Serial.println((int)humidity);
  }
  else {
    //Serial.print("erro=");  Serial.println(err);
    delay(1000);
    return;
  }

  if (humidity >= 40 and runFan == false){
    digitalWrite(pinRELAY, HIGH);
    Serial.write("FON\n");
    runFan = true;
    }
   else if (humidity < 25 and runFan == true) {
    digitalWrite(pinRELAY, LOW);
    Serial.write("FOF\n");
    runFan = false;
    }
  delay(1500);  /* DHT11 sampling rate is 1HZ. */
}
