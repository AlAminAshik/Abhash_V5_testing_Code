#include <NewPing.h>
#include <Arduino.h>
#include <WiFi.h>

const char *ssid     = "Alamin";    //enter name of wifi
const char *password = "12345678";      //enter password for wifi
const char* host = "www.google.com";    //website to check for connectivity
WiFiClient client;
// 59 89 103

#define TRIG_L 16   //pin 27
#define TRIG_F 4    //pin 26 
#define TRIG_R 17   //pin 28
#define pedestrian_led 13  //pedestrian led transistor connected pin 16

const int CH_PD_PIN = 0;          // pin 25 GPIO 0
const int PWR_BUTTON_PIN = 19;    // pin 31
const int Bottom_BUTTON_PIN = 18; // pin 30
const int TOP_BUTTON_PIN = 21;    // pin 33
const int battery_volt_pin = 36;  // pin 4

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
  Serial.println("Device is ON");
    
  WiFi.begin(ssid, password);           //connect to wifi
  pinMode(pedestrian_led, OUTPUT);      //pedestrian led
  pinMode(Bottom_BUTTON_PIN, INPUT_PULLUP); // Middle button with pull-up
  pinMode(TOP_BUTTON_PIN, INPUT_PULLUP); // Top button with pull-up
  pinMode(battery_volt_pin, INPUT);    // Battery voltage pin
  pinMode(2, OUTPUT);                  // on board led
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
  //connect to wifi
  if(WiFi.status() != WL_CONNECTED){ //untill wifi is connected and counting is less than 5, keep trying to connect
    Serial.println("Disconnected");
    delay(500);
    digitalWrite(2, HIGH);     //keep off on board led if not connected to net
  }
  else{
    digitalWrite(2,LOW);   //glow on board led when connection is successful
  }

  //check if internet is available
  if(client.connect(host,80)){
    Serial.println("internet is available");
    client.stop();
  }

  // Power off schedule
  if(digitalRead(PWR_BUTTON_PIN) == LOW) {  //if button is pressed when device is active
      Serial.println("power Button Pressed");
      unsigned long currentMillis = millis(); // get the current time, so we can compare it with the previous time
      if(currentMillis - previousMillis_PWR_Butt > 4000) { //if button pressed for 5 seconds
        while(digitalRead(PWR_BUTTON_PIN) == LOW){    //while button is kept pressed
          Serial.println("Shutting Down");
        }
        digitalWrite(CH_PD_PIN, HIGH);            // turns the mosfet off and thus device off
      }
    }
    else{ 
        previousMillis_PWR_Butt = millis();  //if button is not pressed, store the current time
      }
  
  //check if middle button is pressed
  if(digitalRead(Bottom_BUTTON_PIN) == LOW) {  //if middle button is pressed
    Serial.println("Bottom Button Pressed");
    digitalWrite(pedestrian_led, HIGH);  //dim pedestrian led
  }

  //check if top button is pressed
  if(digitalRead(TOP_BUTTON_PIN) == LOW) {  //if top button is pressed
    Serial.println("Top Button Pressed");
    digitalWrite(pedestrian_led, LOW);  //glow pedestrian led
  }

  //check battery voltage
  int battery_volt = analogRead(battery_volt_pin);  //read battery voltage
  float battery_volt_float = battery_volt * (3.45 / 4095) * 2;  //convert battery voltage to float
  Serial.println("Battery Voltage: " + String(battery_volt_float) + "V");

  getSensorData();
  delay(1000);
}