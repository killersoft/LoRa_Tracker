//HARDIRON WHITEBOX = [-72.30, 37.50], [-68.25, 43.80], [0, 93]

#include <LoRa.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_NeoPixel.h>
#include <TinyGPS++.h>
#include <FlashStorage.h>

#define PIN 2 //What pin the data pin is of the neopixel ring
#define LED_COUNT 24 //Number of LEDs in Neopixel ring
#define NUM_LEDS 24  //Count of the number of LEDs on the neopixel ring 0 to 23 technically !!
#define SECRET_CODE "ABCDEF" //Activate code from same units.

const int buttonPin = 1;  // the number of the pushbutton pin

double Remote_unit_Latatude = -0.0; // Set as default
double Remote_unit_Longitude = 0.0; // Set as default

float hardiron_calibration[3][2] = {{0, 0}, {0, 0}, {0, 0}};

typedef struct {
  boolean valid;
  float x_min;
  float x_max;
  float y_min;
  float y_max;
  float z_min;
  float z_max;
} mem_store_hardiron;

FlashStorage(my_flash_store, mem_store_hardiron);



// Reserve a portion of flash memory to store an "int" variable
// and call it "my_flash_store".
mem_store_hardiron hardiron;


float frequency = 915E6; // E
int txpower=17; // Set transmission power to max
int spreading=10; // Set spreading factor to 12 for maximum range
int coding=4; // 4 to 8 , 8 = for maximum error correction
long bandWidth=31200;// Set signal bandwidth to 125 kHz Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3.
int preamble=8; // Set preamble length to 8 for better signal detection

int gps_valid=0;
int gps_sync_led_loop=0;
int compass_led_index=0;
float compass_heading;
float Remote_unit_bearings;
int Remote_unit_led_index=0;
int pixel_grid[24][3];
double my_lat;
double my_lon;

// Variables
unsigned long buttonPressTime = 0;  // When the button was pressed
const unsigned long longPressTime = 5000;  // Time button needs to be pressed for long press (in ms)
bool buttonActive = false;  // Is the button currently being pressed?

TinyGPSPlus gps; // The TinyGPSPlus object
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(24, PIN, NEO_GRB + NEO_KHZ800); // Parameter 1 = number of pixels in strip Parameter 2 = pin number (most are valid)
sensors_event_t event;

unsigned long lastTime = 0;
const long baseInterval = 25000;  // 20 +/-10sec || seconds in milliseconds



void setup(void)
{

  pinMode(buttonPin, INPUT);  // Setup the buttonPin as input  
  randomSeed(analogRead(0)); // Use an unconnected analog pin to generate random seed
  mag.begin(); //Enable the Compass
  Serial1.begin(9600);
  strip.begin(); //Enable the Neopixel 24 led ring
  strip.setBrightness(10); //adjust neopixel ring brightness
  strip.show(); // turn on neopixel ring...
  Serial.begin(9600); //Enable serial comms to MKR board
  delay(2000);
  if (!LoRa.begin(frequency)) {
    Serial.println("Starting LoRa failed!");
  while (1);
  }
  LoRa.setTxPower(txpower);
  LoRa.setSpreadingFactor(spreading);
  LoRa.setSignalBandwidth(bandWidth);
  LoRa.setCodingRate4(coding);
  LoRa.setPreambleLength(preamble);
  clearPixelGrid();
  //Next must be at bottom of setup.
  
  hardiron = my_flash_store.read();
  if (hardiron.valid == false) {
    calibrate();
    }else{
      hardiron_calibration[0][0] = hardiron.x_min;
      hardiron_calibration[0][1] = hardiron.x_max;
      hardiron_calibration[1][0] = hardiron.y_min;
      hardiron_calibration[1][1] = hardiron.y_max;
      hardiron_calibration[2][0] = hardiron.z_min;
      hardiron_calibration[2][1] = hardiron.z_max;
      Serial.println("**mem **" );
      Serial.println(hardiron_calibration[0][0]);
      Serial.println(hardiron_calibration[0][1]);
      Serial.println(hardiron_calibration[1][0]);
      Serial.println(hardiron_calibration[1][1]);
      Serial.println(hardiron_calibration[2][0]);
      Serial.println(hardiron_calibration[2][1]);
    }
}

void loop(void) 
{
  clearPixelGrid();
  CheckPushButton();
  GetMagneticNorth();
  RXLoRa();
  GetGPS();  
  GetRemoteCoordinates();
  GetMagneticNorth();
  UpdateDisplay();

  unsigned long currentTime = millis();
  long randomizedInterval = baseInterval + random(-10000, 10000); // plus or minus up to 10 seconds

  if (currentTime - lastTime >= randomizedInterval) {
    TXLoRa(); // Call your function here
    lastTime = currentTime;
  }
}

void CheckPushButton(){
  int buttonState = digitalRead(1);  // Read the state of the button
  if (buttonState == HIGH) {  // Button is pressed
    if (buttonActive == false) {  // Was not being pressed before
      buttonActive = true;  // Now we know it's being pressed
      buttonPressTime = millis();  // Record when it started being pressed
    } else {  // Was being pressed before
      if ((millis() - buttonPressTime) > longPressTime) {  // Has been pressed for more than 5 seconds
        calibrate();  // CALIBRATE MODE
        buttonActive = false;  // Reset
      }
    }
  } else {  // Button is not pressed
    buttonActive = false;  // Reset
  }  
}

void longPress() {
  
}


void TXLoRa() {
  //Serial.println("LoRa TX Occured!");
  String message = String(SECRET_CODE) + ',' + String(gps.location.lat(), 6) + ',' + String(gps.location.lng(), 6);
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();  
}

void RXLoRa(){ //Get LoRa signal from other unit
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  String incoming = "";
  if (packetSize) {
        while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }

    if (incoming.startsWith(SECRET_CODE)) {
      incoming = incoming.substring(strlen(SECRET_CODE)+1); // Remove the secret code from the message
      int commaIndex = incoming.indexOf(',');
      String Rx_Latatude = incoming.substring(0, commaIndex);
      String Rx_Longitude = incoming.substring(commaIndex+1);
      
      Remote_unit_Latatude = Rx_Latatude.toDouble();
      Remote_unit_Longitude = Rx_Longitude.toDouble();

     // Serial.print("Latitude: ");
     // Serial.println(Remote_unit_Latatude, 6);

     // Serial.print("Longitude: ");
    //  Serial.println(Remote_unit_Longitude, 6);

    // Serial.print("' with RSSI ");
   // Serial.println(LoRa.packetRssi());
    }
  }
}

void clearPixelGrid() { // Set all led's RGB values to 0, ready for new data;
  for (int i = 0; i < LED_COUNT; i++) {
    for (int j = 0; j < 3; j++) {
      pixel_grid[i][j] = 0;
    }
  }
}

void UpdateDisplay(){  //Update the Neopixel
  int R, G, B =0;
  strip.clear();
   
  for (int i = 0; i < LED_COUNT; i++) { //Loop throu the LED and adjust each unit as required
    R=pixel_grid[i][0];
    G=pixel_grid[i][1];
    B=pixel_grid[i][2];
    strip.setPixelColor(i, R, G, B);  
  }
  strip.show(); // Update the LED strip with the new color data
}

void GetRemoteCoordinates(){
  if (Remote_unit_Latatude!=0){
    int ledPos = map(Remote_unit_bearings, 0, 360, 0, NUM_LEDS);
    Remote_unit_led_index = (ledPos + compass_led_index) % NUM_LEDS;
    addpixel(Remote_unit_led_index, 0, 255, 0 );    
    }else{
    //If not valid..... something
  }
}

void GetMagneticNorth(){  
  mag.getEvent(&event); 
  float magvals[3] = {event.magnetic.x, event.magnetic.y, event.magnetic.z};
  float normvals[3];  //UPDATED MAGNETIC COMPASS RESULTS
  normalize(magvals, normvals); // FIX MAGNETIC COMPASS HARD IRON OFFSETS
  compass_heading = atan2(normvals[1], normvals[0]) * 180.0 / PI; // we will only use X and Y for the compass calculations, so hold it level!
  compass_heading += 180; // compass_heading is between -180 and +180 since atan2 returns -pi to +pi, this translates it to be between 0 and 360
  compass_led_index = altLedIndex(compass_heading);  //GET LED FOR COMPASS NORTH
  addpixel(compass_led_index,255,0,0); //Add the new north to the compass grid.
}

int addpixel(int led, int R, int G, int B){ //add a rgb value to a neopixel index  
  pixel_grid[led][0] = R; 
  pixel_grid[led][1] = G;
  pixel_grid[led][2] = B;  
}

int getLedIndex(float bearingx) {// Set 
  if (bearingx > 360) {
      bearingx -= 360;
  }
  bearingx = constrain(bearingx, -7.5, 352.5);  // Limit the bearing to the range -7.5 to 352.5
  bearingx += 7.5;  // Add 7.5 to the bearing to shift the range to 0 to 360

  // Map the bearing to an LED index in steps of 15 degrees
  //int index = round(bearing / 15.0) % NUM_LEDS;
  float index = round(bearingx / 15.0);
  index = fmod(index, NUM_LEDS);
  
  return index;
}

void GetGPS(){

  while (Serial1.available() > 0) {
    if(gps.encode(Serial1.read()));
      if (gps.location.isValid()) {
        gps_valid=1;
        my_lat = gps.location.lat();
        my_lon = gps.location.lng();
        //Serial.print("My Latitude: ");
        //Serial.println(my_lat, 6);

        //Serial.print("My Longitude: ");
        //Serial.println(my_lon, 6);
        //Serial.print("Remote Latitude: ");
        //Serial.println(Remote_unit_Latatude, 6);

        //Serial.print("Remote Longitude: ");
        //Serial.println(Remote_unit_Longitude, 6);
      
        Remote_unit_bearings = gps.courseTo(my_lat, my_lon, Remote_unit_Latatude, Remote_unit_Longitude); 
        break;
        } else {
          gps_valid=0;
          if(gps_sync_led_loop>23){
            gps_sync_led_loop=0;
         }
        addpixel(gps_sync_led_loop, 0, 0, 255 );
        gps_sync_led_loop++;
        break;    
        }
  }
}

void normalize(float magvals[3], float ret[3]) {  //COMPASS CALIBRATION CHANGES
  for (int i = 0; i < 3; i++) {
    float minv = hardiron_calibration[i][0];
    float maxv = hardiron_calibration[i][1];
    float axis = magvals[i];

    axis = min(max(minv, axis), maxv);  // keep within min/max calibration
    ret[i] = (axis - minv) * 200 / (maxv - minv) - 100;
  }
}

int altLedIndex(float bearing) {
  int result = 0;

  // Normalize the bearing to a value between 0 and 360
  bearing = bearing < 0 ? bearing + 360 : bearing;
  bearing = bearing >= 360 ? bearing - 360 : bearing;

  // Your original code for mapping bearings to LED indices
  if ((bearing >= 352.5) || (bearing < 7.5)) {
    result = 0;
  } else if ((bearing >= 7.5) && (bearing < 22.5)) {
    result = 1;
  } else if ((bearing >= 22.5) && (bearing < 37.5)) {
    result = 2;
  } else if ((bearing >= 37.5) && (bearing < 52.5)) {
    result = 3;
  } else if ((bearing >= 52.5) && (bearing < 67.5)) {
    result = 4;
  } else if ((bearing >= 67.5) && (bearing < 82.5)) {
    result = 5;
  } else if ((bearing >= 82.5) && (bearing < 97.5)) {
    result = 6;
  } else if ((bearing >= 97.5) && (bearing < 112.5)) {
    result = 7;
  } else if ((bearing >= 112.5) && (bearing < 127.5)) {
    result = 8;
  } else if ((bearing >= 127.5) && (bearing < 142.5)) {
    result = 9;
  } else if ((bearing >= 142.5) && (bearing < 157.5)) {
    result = 10;
  } else if ((bearing >= 157.5) && (bearing < 172.5)) {
    result = 11;
  } else if ((bearing >= 172.5) && (bearing < 187.5)) {
    result = 12;
  } else if ((bearing >= 187.5) && (bearing < 202.5)) {
    result = 13;
  } else if ((bearing >= 202.5) && (bearing < 217.5)) {
    result = 14;
  } else if ((bearing >= 217.5) && (bearing < 232.5)) {
    result = 15;
  } else if ((bearing >= 232.5) && (bearing < 247.5)) {
    result = 16;
  } else if ((bearing >= 247.5) && (bearing < 262.5)) {
    result = 17;
  } else if ((bearing >= 262.5) && (bearing < 277.5)) {
    result = 18;
  } else if ((bearing >= 277.5) && (bearing < 292.5)) {
    result = 19;
  } else if ((bearing >= 292.5) && (bearing < 307.5)) {
    result = 20;
  } else if ((bearing >= 307.5) && (bearing < 322.5)) {
    result = 21;
  } else if ((bearing >= 322.5) && (bearing < 337.5)) {
    result = 22;
  } else if ((bearing >= 337.5) && (bearing < 352.5)) {
    result = 23;
  }

  return result;
}

void calibrate(){
  unsigned long startTime = millis();
  strip.clear();
  for(int x=0; x<24;x++){
      strip.setPixelColor(x, 255,255,255);  
    }
    strip.show();
  // Update the high and low extremes
  while ((millis() - startTime) < 15000) {  // 15 seconds
    /* Get a new sensor event */
    
    sensors_event_t magEvent;
    mag.getEvent(&magEvent);

    // Update calibration values
    hardiron_calibration[0][0] = min(hardiron_calibration[0][0], magEvent.magnetic.x);
    hardiron_calibration[0][1] = max(hardiron_calibration[0][1], magEvent.magnetic.x);

    hardiron_calibration[1][0] = min(hardiron_calibration[1][0], magEvent.magnetic.y);
    hardiron_calibration[1][1] = max(hardiron_calibration[1][1], magEvent.magnetic.y);

    hardiron_calibration[2][0] = min(hardiron_calibration[2][0], magEvent.magnetic.z);
    hardiron_calibration[2][1] = max(hardiron_calibration[2][1], magEvent.magnetic.z);

    Serial.print("Calibrating - X:"); Serial.print(magEvent.magnetic.x); 
    Serial.print(", Y:"); Serial.print(magEvent.magnetic.y); 
    Serial.print(", Z:"); Serial.println(magEvent.magnetic.z);
  }

  Serial.println("Calibration complete:");
  for (int i = 0; i < 3; i++) {
    Serial.print("Axis "); Serial.print(i); Serial.print(": ");
    Serial.print("Min = "); Serial.print(hardiron_calibration[i][0]); Serial.print(", ");
    Serial.print("Max = "); Serial.println(hardiron_calibration[i][1]);
  }
  hardiron.valid = true;
  hardiron.x_min = hardiron_calibration[0][0];
  hardiron.x_max = hardiron_calibration[0][1];
  hardiron.y_min = hardiron_calibration[1][0];
  hardiron.y_max = hardiron_calibration[1][1];
  hardiron.z_min = hardiron_calibration[2][0];
  hardiron.z_max = hardiron_calibration[2][1];
  my_flash_store.write(hardiron);
}
