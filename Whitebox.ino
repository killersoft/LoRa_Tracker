/*
 * LORA TRACKING DEVICE.
 * DESIGNED AND WRITTEN BY GREGORY MCLENNAN 2023
 * greg@gregmc.net
 */
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

double Remote_unit_Latatude = -0.0; // Set as default lat until real values are determined
double Remote_unit_Longitude = 0.0; // Set as default long until real values are determined

float hardiron_calibration[3][2] = {{0, 0}, {0, 0}, {0, 0}};  //Set default values for hard iron offsets, these will change when calibration values are loaded from memory.

typedef struct {  // A structure to hold hariron offsets, specifically for reading/writing from flash memory
  boolean valid;
  float x_min;
  float x_max;
  float y_min;
  float y_max;
  float z_min;
  float z_max;
} mem_store_hardiron;  //Called it mem_store_hardiron !

FlashStorage(my_flash_store, mem_store_hardiron); // Get the struct from the 'my_flash_store' storage of nvram !

// and call it "my_flash_store".
mem_store_hardiron hardiron;  //Make a type of hardiron stuctire called mem_store_hardiron.


float frequency = 915E6; // The E is an exponential, 915E6 = 915000000Hz || 433E6 = 433000000Hz frequency RF.
int txpower=23; // Set transmission power in dBm value.
int spreading=7; // Set spreading factor to 12 for maximum range. Specific to LoRa ( chirp rate )
int coding=8; // 5 to 8 , 8 = for maximum error correction 
long bandWidth=125E3;// Set signal bandwidth to 125 kHz Supported values are 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, and 500E3.
int preamble=8; // Set preamble length to 8 for better signal detection

//GLOBAL Variables.. I know, I know.. but its works ok for embedded stuff !

int gps_valid=0; //Gets set when GPS signal is in working state.
int gps_sync_led_loop=0;  // Keep track of loss of GPS signal LED indicator 0 -> 23
int compass_led_index=0;  // Which LED is lit to show user what direction Magnetic North is.
float compass_heading;  //Keeps the magnetic north current bearing offset value. 
float Remote_unit_bearings; //When we receive a lat/long message from the other unit, we convert current angle bearings offset. 
int Remote_unit_led_index=0; // The LED used to show where the other unit is located
int pixel_grid[24][3];  //The neopixel grid [LED NUMBER 0 TO 23] [RED(0-255), GREEN(0-255), BLUE(0-255)]
double my_lat; //Holds GPS lat data
double my_lon; // Holds GPS long data

unsigned long buttonPressTime = 0;  // Timer value for when the button was pressed for calibration activation
const unsigned long longPressTime = 5000;  // Time button needs to be pressed for long press (in ms)
bool buttonActive = false;  // Is the button currently being pressed?

TinyGPSPlus gps; // The TinyGPSPlus object
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);  // Make 'mag' a new object for the magnetic compass
Adafruit_NeoPixel strip = Adafruit_NeoPixel(24, PIN, NEO_GRB + NEO_KHZ800); // Parameter 1 = number of pixels in strip Parameter 2 = pin number (most are valid)
sensors_event_t event; // Create a sensors_event_t object in memory to hold results

unsigned long lastTime = 0; //Timer used for last time LoRa TX was executed
const long baseInterval = 35000;  // 35 +/-10sec || seconds in milliseconds

void setup(void)
{

  pinMode(buttonPin, INPUT);  // Setup the buttonPin as input if user presses and hold it for 5 seconds for magnetic hardiron offset calibration. 
  randomSeed(analogRead(0)); // Use an unconnected analog pin to generate random seed for true randomization.
  mag.begin(); //Enable the Compass
  Serial1.begin(9600); //Enable the GPS serial port
  strip.begin(); //Enable the Neopixel 24 led ring
  strip.setBrightness(20); //adjust neopixel ring brightness
  strip.show(); // turn on neopixel ring...
  Serial.begin(9600); //Enable serial comms to MKR board via usb
  if (!LoRa.begin(frequency)) { //Set LoRa frequncy
    Serial.println("Starting LoRa failed!");
  }
  LoRa.setTxPower(txpower); //Set transmit power level
  LoRa.setSpreadingFactor(spreading); //Set LoRa chirp spread
  LoRa.setSignalBandwidth(bandWidth); //Set bandwidth ( lower the further/higest reliability, but lower speed )
  LoRa.setCodingRate4(coding); // Error correction rate values 4-8
  LoRa.setPreambleLength(preamble); //For receiver to lock on prior to main transmission
  LoRa.setGain(6); 
  clearPixelGrid(); // Clear the neopixel grid

  
  hardiron = my_flash_store.read(); // Read the hardiron offsets in memory
  if (hardiron.valid == false) { //Well if the unit was just flashed with new code, there wont be values in memory, so start the calibration process.
    calibrate(); // Call calibrate mode
    }else{ //There is some valid data in the memory read so set the hardiron offsets
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

void loop(void) //Main loop
{
  clearPixelGrid(); // Clear the array for the neopixels
  RXLoRa(); //See if the LoRa unit has received a message from the other unit
  CheckPushButton(); //See if user has activated the pushbutton for a recalibrate of the magnetic compass for hardiron offset
  GetMagneticNorth(); //Get the bearing offset for Magnetic north according to the sensor
  GetGPS(); //See if onboard GPS has a lock, if so get current co-ordinates
  GetRemoteCoordinates(); //If we did receive loc data from remote device, update the local LED to point to their direction
  UpdateDisplay();  //Update the 24 led neopixel with updated data from any of the above.
  TimerTXLoRa(); //Test to see if we are in the transmit window time and send/TX our data to remote unit
}

void TimerTXLoRa(){
  
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

void TXLoRa() {
  //Serial.println("LoRa TX Occured!");
  String message = String(SECRET_CODE) + ',' + String(gps.location.lat(), 6) + ',' + String(gps.location.lng(), 6);
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket(true);  //True makes it NON blocking ASYNC MODE....
  Serial.println("Transmitted");
}

void RXLoRa(){ //Get LoRa signal from other unit
  // try to parse packet
  int packetSize = LoRa.parsePacket(); //Sizeof received packet
  String incoming = ""; //clear a new string for received sentance
  if (packetSize) { //packetsize > null
        while (LoRa.available()) { //
      incoming += (char)LoRa.read(); //Read 
    }

    if (incoming.startsWith(SECRET_CODE)) {
      incoming = incoming.substring(strlen(SECRET_CODE)+1); // Remove the secret code from the message
      int commaIndex = incoming.indexOf(',');
      String Rx_Latatude = incoming.substring(0, commaIndex);
      String Rx_Longitude = incoming.substring(commaIndex+1);
      
      Remote_unit_Latatude = Rx_Latatude.toDouble();
      Remote_unit_Longitude = Rx_Longitude.toDouble();
      
      strip.setBrightness(100);


      Serial.print("RSSI : ");
      Serial.println(LoRa.packetRssi());
    }
  }
}

void clearPixelGrid() { // Set all led's RGB values to 0, ready for new data;
  for (int i = 0; i < LED_COUNT; i++) {
    for (int j = 0; j < 3; j++) {
      pixel_grid[i][j] = 0; //Set 0 for no led light
    }
  }
  strip.setBrightness(20);
}

void UpdateDisplay(){  //Update the Neopixel LED's using the pixel_grid[][] array
  int R, G, B =0;
  strip.clear();
   
  for (int i = 0; i < LED_COUNT; i++) { //Loop throu the LED and adjust each unit as required
    R=pixel_grid[i][0];
    G=pixel_grid[i][1];
    B=pixel_grid[i][2];
    strip.setPixelColor(i, R, G, B);  
  }
  strip.show(); // Update the LED strip with the new color data and show !
}

void GetRemoteCoordinates(){
  if (Remote_unit_Latatude!=0){ //If remote unit is valid and not the default 0,0 lat/long !
    int ledPos = map(Remote_unit_bearings, 0, 360, 0, NUM_LEDS); //ID what LED should be lit to ID remote device
    Remote_unit_led_index = (ledPos + compass_led_index) % NUM_LEDS; //Knowing what LED should be lit, then offset it by the North LED as the device may be rotated
    addpixel(Remote_unit_led_index, 0, 255, 0 );  //Add the new offsetted Remote device LED  
    }else{
    //If not valid.....
    //Nothing needs to go here unless you want to action bad co-ordinates of remote device.
    //Even if nothing, the last know good value of the remote unit remains in place...
  }
}

void GetMagneticNorth(){  
  mag.getEvent(&event); //Get compass data
  float magvals[3] = {event.magnetic.x, event.magnetic.y, event.magnetic.z}; //Update the array with X,Y,Z values from the compass
  float normvals[3];  //UPDATED MAGNETIC COMPASS RESULTS
  normalize(magvals, normvals); // FIX MAGNETIC COMPASS HARD IRON OFFSETS
  compass_heading = atan2(normvals[1], normvals[0]) * 180.0 / PI; // we will only use X and Y for the compass calculations, so hold it level!
  compass_heading += 180; // compass_heading is between -180 and +180 since atan2 returns -pi to +pi, this translates it to be between 0 and 360
  compass_led_index = altLedIndex(compass_heading);  //GET LED INDEX TO DISPLAY WHAT COMPASS NORTH IS GOING TO BE LIT.
  addpixel(compass_led_index,255,0,0); //Add the new north LED to the compass array grid for later processing
}

int addpixel(int led, int R, int G, int B){ //add a R,G,B value to a neopixel index to combine any updates for a led array cell.
   
  if((pixel_grid[led][0] + R)>255 ){
    pixel_grid[led][0]= 255; 
  }else{
    pixel_grid[led][0] = pixel_grid[led][0] + R; 
  }
   if((pixel_grid[led][1] + G)>255 ){
    pixel_grid[led][1]= 255; 
  }else{
    pixel_grid[led][1]=pixel_grid[led][1] + G; 
  }
  if((pixel_grid[led][2] + B)>255 ){
    pixel_grid[led][2]= 255; 
  }else{
    pixel_grid[led][2]=pixel_grid[led][2]+ B; 
  } 
}

void GetGPS(){

  while (Serial1.available() > 0) { //Need this to do a full RX of buffers 
    if(gps.encode(Serial1.read())); //Get ALL the NEMA sentences
      if (gps.location.isValid()) { //We all like valid data !
        gps_valid=1; //Data must be valid so set the global variable for other functions to act on
        my_lat = gps.location.lat(); //Get the devices latitude
        my_lon = gps.location.lng(); //Get the devices longtitude
     
        Remote_unit_bearings = gps.courseTo(my_lat, my_lon, Remote_unit_Latatude, Remote_unit_Longitude); 
        break; // Break the while loop!
        } else {
          gps_valid=0; //Set the valid state to 0
          if(gps_sync_led_loop>23){ // As part of telling the user the GPS data is not synced
            gps_sync_led_loop=0; //If gps_sync_led_loop > 23 then make gps_sync_led_loop=0
         }
        addpixel(gps_sync_led_loop, 0, 0, 255 ); //Set the looping color to blue
        gps_sync_led_loop++; //Increment the counter for the looped LED's
        break;    //Again break the while loop
        }
  }
}

void normalize(float magvals[3], float ret[3]) {  //Take RAW magnetic compass data for the X, Y, Z planes, and apply hardiron offset data(Min/Max) for each direction to correct for magnetic variances.
  for (int i = 0; i < 3; i++) {
    float minv = hardiron_calibration[i][0];
    float maxv = hardiron_calibration[i][1];
    float axis = magvals[i];

    axis = min(max(minv, axis), maxv);  // keep within min/max calibration
    ret[i] = (axis - minv) * 200 / (maxv - minv) - 100;
  }
}

int altLedIndex(float bearing) {  //Lets MAP out received bearing to LED's locations as part of a compass.
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

void calibrate(){ // Calibrate for hardiron offsets to get best effort to determine where magnetic north is located. Best completed outdoors 1.5 meters away from any structures.
  unsigned long startTime = millis(); //As part of the 15 seconds worth of calibration
  strip.clear(); // Clear neopixel data.
  for(int x=0; x<24;x++){ //Neopixel array
      strip.setPixelColor(x, 255,255,255);  //Fill neopixel with all white
    }
    strip.show();//Show neopixel values
  // Update the high and low extremes
  while ((millis() - startTime) < 15000) {  // 15 seconds of calibration.
    /* Get a new sensor event */
    
    sensors_event_t magEvent; //Store events
    mag.getEvent(&magEvent); //get magnetic events from memory location of magevent.

    // Update calibration values for all X, Y, Z sensors
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
  hardiron.valid = true; // Update the structure for a true result
  hardiron.x_min = hardiron_calibration[0][0]; 
  hardiron.x_max = hardiron_calibration[0][1];
  hardiron.y_min = hardiron_calibration[1][0];
  hardiron.y_max = hardiron_calibration[1][1];
  hardiron.z_min = hardiron_calibration[2][0];
  hardiron.z_max = hardiron_calibration[2][1];
  my_flash_store.write(hardiron); //Update the flash memory of the device with the new calibration data
}
