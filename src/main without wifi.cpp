#include <NewPing.h>
#include <Arduino.h>

//#define TRIG_F 12  //D6 //2  //wemos d1 mini
#define TRIG_F 4   //D4 //26 //nodemcu esp32

#define MAX_DISTANCE_FRONT 184

NewPing sonarF(TRIG_F, TRIG_F, MAX_DISTANCE_FRONT);

int dF;

void setup() {
  Serial.begin(115200);
  Serial.println("Start");
}

long dst_median(NewPing s, int max, int cnt) {
  long tmr = millis();
  long dstt = 1;
  for (int i = 0; i < cnt; i++) {
    long d = s.ping_cm();
    //dstt += s.ping_cm();
    if (d == 0) {
      d = max;
    }
    dstt = dstt * d;
  }
  //dstt = dstt / cnt;
  dstt = round(pow(dstt, 1.0 / cnt));
  Serial.println(millis() - tmr);
  return dstt;
}

void getSensorData() {
  dF = dst_median(sonarF, MAX_DISTANCE_FRONT, 4);
  delay(10);
  Serial.println("Front: " + String(dF) + "cm");
}

void loop(){
  getSensorData();
  delay(1000);
}