#include <NewPing.h>
#include <Arduino.h>

//#define TRIG_F 12  //D6 //2  //wemos d1 mini
#define TRIG_L 16  //pin 27
#define TRIG_F 4  //pin 26 
#define TRIG_R 17  //pin 28

#define MAX_DISTANCE_FRONT 184
#define MAX_DISTANCE_SIDE 184

NewPing sonarL(TRIG_L, TRIG_L, MAX_DISTANCE_SIDE + 30);
NewPing sonarF(TRIG_F, TRIG_F, MAX_DISTANCE_FRONT + 30);
NewPing sonarR(TRIG_R, TRIG_R, MAX_DISTANCE_SIDE + 30);

int dL, dF, dR;

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
  dF = dst_median(sonarF, MAX_DISTANCE_FRONT + 30, 4);
  delay(10);
  dL = dst_median(sonarL, MAX_DISTANCE_SIDE + 30, 4);
  delay(10);
  dR = dst_median(sonarR, MAX_DISTANCE_SIDE + 30, 4);
  delay(10);
  Serial.println("Distance: Left: " + String(dL) + "cm - Front: " + String(dF) + "cm - Right: " + String(dR) + "cm");
}

void loop(){
  getSensorData();
  delay(1000);
}