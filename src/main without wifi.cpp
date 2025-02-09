#include <NewPing.h>
#include <Arduino.h>

//#define TRIG_F 12  //D6 //2  //wemos d1 mini
#define TRIG_L 16  //pin 27
#define TRIG_F 4  //pin 26 
#define TRIG_R 17  //pin 28

const int CH_PD_PIN = 0;          // pin 25 GPIO 0
const int PWR_BUTTON_PIN = 19;    // pin 31

unsigned long previousMillis_PWR_Butt = 0;  // will store last time button has pressed

#define MAX_DISTANCE_FRONT 184
#define MAX_DISTANCE_SIDE 184
NewPing sonarL(TRIG_L, TRIG_L, MAX_DISTANCE_SIDE + 30);
NewPing sonarF(TRIG_F, TRIG_F, MAX_DISTANCE_FRONT + 30);
NewPing sonarR(TRIG_R, TRIG_R, MAX_DISTANCE_SIDE + 30);
int dL, dF, dR;



void setup() {
  Serial.begin(115200);
  Serial.println("Start");

  //Power on schedule
  pinMode(PWR_BUTTON_PIN, INPUT_PULLUP);  // Button with pull-up
  pinMode(CH_PD_PIN, OUTPUT);         // sets GPIO0 as an output which will pull the mosfet ON
  digitalWrite(CH_PD_PIN, HIGH);      // keep the mosfet off
  delay(4000);                        // wait for a defined delay time the device to boot up
  digitalWrite(CH_PD_PIN, LOW);       // latches the transistor to keep device on
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
  // Power off schedule
  if(digitalRead(PWR_BUTTON_PIN) == LOW) {  //if button is pressed when device is active
    unsigned long currentMillis = millis(); // get the current time, so we can compare it with the previous time
    if(currentMillis - previousMillis_PWR_Butt > 5000) { //if button pressed for 5 seconds
      while(digitalRead(PWR_BUTTON_PIN) == LOW){    //while button is kept pressed
        //sound power down
      }
      digitalWrite(CH_PD_PIN, HIGH);            // turns the mosfet off and thus device off
    }
  }
  else{ 
    previousMillis_PWR_Butt = millis();  //if button is not pressed, store the current time
  }

  getSensorData();
  delay(1000);
}