
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/
// Analog input pin that the Velostat is connected to



/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

const int S1_HEELINPIN = A1;
const int S1_SOLEPIN = A2;

const int S2_HEELINPIN = A4;
const int S2_SOLEPIN = A5;

const int BLE_READ_TIMEOUT = 100;

const int AVERAGE_READING = 500;

int PIN_OUTS[] = {S1_HEELINPIN, S1_SOLEPIN, 
                  S2_HEELINPIN, S2_SOLEPIN};

const int SENSORS_PER_FEET = 4;
const int MAX_MILISECONDS_IDLE = 60 * 1000; //Express in millis.

int MILISECONDS_IDLE = 0;

int SENSOR_THRESHOLD[SENSORS_PER_FEET];

int sensor_readings[SENSORS_PER_FEET];

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);

}

void led_alert(int time){
   digitalWrite(LED_BUILTIN, HIGH);
   delay(time);  
   digitalWrite(LED_BUILTIN, LOW);
}

void setup(void) {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("Setup");
  
  pinMode(S1_HEELINPIN, INPUT);
  pinMode(S1_SOLEPIN, INPUT);
  
  pinMode(S2_HEELINPIN, INPUT);
  pinMode(S2_SOLEPIN, INPUT);
  

  init_sensor_threshold();
  led_alert(500);
  Serial.print("Setup Done");

  for (int i = 0; i < SENSORS_PER_FEET; i++) {
  //Report the average      
  if (Serial){
     Serial.print("Reference value for: [");  
     Serial.print(PIN_OUTS[i]);  
     Serial.print("] is [");
     Serial.print(SENSOR_THRESHOLD[i]);
     Serial.println("]"); 
  }
  }


  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  digitalWrite(LED_BUILTIN, HIGH); 

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(600);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }
  
}


void init_sensor_threshold() {
  for (int i = 0; i < SENSORS_PER_FEET; i++) {
     long sum = 0;
     for (int j = 0; j < AVERAGE_READING; j++) {
         int reading = 0;
         if (i < 4) {
           reading = analogRead(PIN_OUTS[i]);
         }else{
           reading = digitalRead(PIN_OUTS[i]);
         }
         //sum of all the readings 
         sum = sum + reading;
         delay(10);
     }  
     //Get the average so this is the top of the 
     SENSOR_THRESHOLD[i] = ((sum / AVERAGE_READING) * 3) >> 2;   
     
  }
  
}

void readSensors(void){
  for(int i = 0; i < SENSORS_PER_FEET; i++ ){
        int reading  = 0;
       if (i < 4) {
           reading = analogRead(PIN_OUTS[i]);
         }else{
           reading = digitalRead(PIN_OUTS[i]);
         }
       sensor_readings[i] = ((sensor_readings[i] * 3) + reading) >> 2;
    }
}


bool getUserInput(char buffer[], uint8_t maxSize)
{
  // timeout in 100 milliseconds
  TimeoutTimer timeout(BLE_READ_TIMEOUT);

  memset(buffer, 0, maxSize);
  while( (!Serial.available()) && !timeout.expired() ) { delay(1); }

  if ( timeout.expired() ) {   
    return false;
  }

  delay(2);
  uint8_t count=0;
  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && (Serial.available()) );

  return true;
}

void readCommand(void){
  // Check for user input
  char inputs[BUFSIZE+1];

  //This segment reads input from monitor and sent it back to the device
  if ( getUserInput(inputs, BUFSIZE) )
  {
    // Send characters to Bluefruit
    Serial.print("[Send] ");
    Serial.println(inputs);

    ble.print("AT+BLEUARTTX=");
    ble.println(inputs);

    // check response stastus
    if (! ble.waitForOK() ) {
      Serial.println(F("Failed to send?"));
    }
  }

  // Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    MILISECONDS_IDLE = MILISECONDS_IDLE + BLE_READ_TIMEOUT;
    return;
  }
  //Info is read lets reset idle counter
  MILISECONDS_IDLE = 0;
  // Some data was found, its in the buffer
  Serial.print(F("[Recv] ")); Serial.println(ble.buffer);

  Serial.println('Wait for OK');
    

  ble.waitForOK();
   
  if (strcmp(ble.buffer, "FOOT_DATA") == 0){
    String reading = String("{\"f\": [");
    for(int i = 0; i < SENSORS_PER_FEET; i++ ){
       //Percentage Read MAX:100::read:X
        Serial.print(F("[Reading] ")); Serial.println(sensor_readings[i]);
       int percentage = (sensor_readings[i] * 100) / SENSOR_THRESHOLD[i];
       if (percentage > 100){
          percentage = 100;
       }
       reading.concat(percentage);
       if (i < SENSORS_PER_FEET - 1){
           reading.concat(",");
        }
    }
    reading.concat("]};");

    Serial.println(reading);
    
    ble.print("AT+BLEUARTTX=");   
    ble.println(reading);  
    // check response stastus
    if (! ble.waitForOK() ) {
      Serial.println(F("Failed to send?"));
    }
  }
  
  
}

/// the loop function runs over and over again for ever
void loop(void) {

   readSensors();
   readCommand();
  
  

  if (MILISECONDS_IDLE >= MAX_MILISECONDS_IDLE){
     
       Serial.println(F("Too Much Idle Time. disconecting Performing Factory Reset"));
      if ( ! ble.factoryReset() ){
        error(F("Couldn't factory reset"));
      }else{
        Serial.println(F("Reset waiting for conection"));
       }

      while (! ble.isConnected()) {
       delay(600);
      } 

      if (ble.isConnected()){
        MILISECONDS_IDLE = 0;
         Serial.println(F("Connected"));
      }
      
  }
}
