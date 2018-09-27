#include <Wire.h>
#include <apds9960.h>
#include <ArduinoJson.h>
DynamicJsonBuffer DynamicJson;
// Pins
#define APDS9960_INT    2 // Needs to be an interrupt pin


// Global Variables
APDS9960 apds = APDS9960();

uint8_t proximity;
uint16_t r,g,b,c;
int i;
String DIR = " ";
void setup() {

  // Set interrupt pin as input
  pinMode(APDS9960_INT, INPUT);

  // Initialize Serial port
  Serial.begin(9600);
  Serial.println();
  Serial.println(F("--------------------------------"));
  Serial.println(F("SparkFun APDS-9960 - GestureTest"));
  Serial.println(F("--------------------------------"));
  
  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.apds9960_init() ) {
    Serial.println(F("APDS-9960 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }
}
  // Start running the APDS-9960 gesture sensor engine
  


void loop() {
  
// Constants

 apds.apds9960_enableGestureSensor(true);
   //apds.apds9960_enableProximitySensor(true);
 //  apds.apds9960_setProximityIntLowThreshold(0);
  // apds.apds9960_setProximityIntHighThreshold(50);
 //  apds.apds9960_setProximityIntEnable(true);
   uint8_t gesture = apds.apds9960_readGesture();
    //uint8_t gesture = apds.readGesture();
    
 if ( gesture == DIR_RIGHT || gesture == DIR_DOWN || gesture == DIR_UP || gesture == DIR_LEFT|| gesture==DIR_NEAR|| gesture==DIR_FAR ){
  
 // apds.apds9960_getGesture();
  apds.apds9960_readProximity(proximity);
  apds.apds9960_enableProximitySensor(false);
  apds.apds9960_enableGestureSensor(false);
  apds.apds9960_enableLightSensor(true);
  delay(5);
  apds.apds9960_readRedLight(r);
  apds.apds9960_readGreenLight(g);
  apds.apds9960_readBlueLight(b);
  apds.apds9960_readAmbientLight(c);
  apds.apds9960_enableLightSensor(false);
  delay(5);  
  
 StaticJsonBuffer<4096> jsonBuffer;
   JsonObject& root = jsonBuffer.createObject();
  JsonArray& rgb = root.createNestedArray("rgb");
  rgb.add(r);
  rgb.add(g);
  rgb.add(b);
  //JsonObject& root2 = jsonBuffer.createObject();
  JsonArray& als = root.createNestedArray("als");
  als.add(c);
  JsonArray& up = root.createNestedArray("up");
  JsonArray& down = root.createNestedArray("down");
  JsonArray& left = root.createNestedArray("left");
  JsonArray& right = root.createNestedArray("right");
  for(int i = 0 ; i < apds.tot_num_gest; i++){
    up.add(apds.u_ptr[i]);
    down.add(apds.d_ptr[i]);
    left.add(apds.l_ptr[i]);
    right.add(apds.r_ptr[i]);
    }
  
    
  if(gesture==1){root["direction"]="left";} 
  else if(gesture==2){root["direction"]="right";}
  else if(gesture==3){root["direction"]="up";}
  else if(gesture==4){root["direction"]="down";}
  else if(gesture==5){root["direction"]="near";}
  else if(gesture==6){root["direction"]="far";}
  else if(gesture==7){root["direction"]="all";}
  else if(gesture==0){root["direction"]="none";}
   root.printTo(Serial);
   Serial.println();
   //root2.printTo(Serial);
   
  
  }}


