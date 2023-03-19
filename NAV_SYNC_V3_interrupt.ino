/*
This is to measure the latency between a light and sound sensors.
We show the result in an OLED display 128x64 I2C with SSD1306 driver using the Adafruit library.



Features:
 
 - Maximum latency: 2 seconds.
 - Precision: 0.001 seconds (1 millisecond)
 
ESP32 USES GPIO 22 (SCL) and GPIO 21 (SDA)

*/
#include <WiFi.h>
#include <BluetoothSerial.h>
//#include "driver/adc.h"
#include <esp_bt.h>
#include <SPI.h>
#include <Wire.h>

#include "AiEsp32RotaryEncoder.h"
#include "OneButton.h"
unsigned long pressStartTime;

#define ROTARY_ENCODER_A_PIN 5
#define ROTARY_ENCODER_B_PIN 17
#define ROTARY_ENCODER_BUTTON_PIN 15 // old value = 16
#define ROTARY_ENCODER_STEPS 4

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);


const unsigned long TimeOutAmount = 3000000;  // Amount to wait after a pulse to wait
                                              // for the other sensor pulse. This value is 0.000001 seconds


//---------------------------------------------------------
//---------------------------------------------------------
//---------------------------------------------------------


//// Variables:
unsigned long CurrentMicros = micros();  // Stores the micros in that cycle.
                                         // We need a variable with a value that is not going to be affected by the interrupt
                                         // because we are going to do math and functions that are going to mess up if the values
                                         // changes in the middle of the cycle.



long Difference;  // Difference between both sensors

long Latency;  // Final latency result in milliseconds

volatile unsigned long Micros_For_Light_Sensor;
volatile unsigned long Micros_For_Mic;

int Offset = 0;

bool done = false;
bool notskip = true;

byte Ready_State;  // Store if we are ready for a new test for both sensors
byte Ready_Light;  // Store if we are ready for a new test for the light
byte Ready_Mic;    // Store if we are ready for a new test for the mic


byte FirstLight;  // To store who is first
byte FirstMic;    // To store who is first

byte WeAreWaiting;  // This is 1 when waiting for a pulse
OneButton button(16, true); //active low button

#include <Adafruit_GFX.h>      // Include core graphics library for the display
#include <Adafruit_SSD1306.h>  // Include Adafruit_SSD1306 library to drive the display


Adafruit_SSD1306 display(128, 64);  // Create display


#include <Fonts/FreeMonoBold18pt7b.h>  // Add a custom font
#include <Fonts/FreeMono12pt7b.h>      // Add a custom font
#include <Fonts/FreeMono9pt7b.h>       // Add a custom font
void IRAM_ATTR readEncoderISR() {
  rotaryEncoder.readEncoder_ISR();
}

void IRAM_ATTR checkTicks() {
  // include all buttons here to be checked
  button.tick(); // just call tick() to check the state.
}

void singleClick() {
  //Serial.println("singleClick() detected.");
  if (notskip) {done = false;
  delay(300);
  }
digitalWrite(26,0);  
} // singleClick


// this function will be called when the button was pressed 2 times in a short timeframe.
void doubleClick() {
  
  notskip = !notskip;
  if (notskip){ 
  digitalWrite(23, 1);
  //Serial.println("notskip = true");
  } 
  else 
  {digitalWrite(23, 0);
  //Serial.println("notskip = false");
  }
  
} // doubleClick


// this function will be called when the button was pressed multiple times in a short timeframe.
void multiClick() {
  int n = button.getNumberClicks();
  if (n == 3) {
    //Serial.println("tripleClick detected.");
  } else if (n == 4) {
    //Serial.println("quadrupleClick detected.");
  } else {
    //Serial.print("multiClick(");
    //Serial.print(n);
    //Serial.println(") detected.");
  }
}
// this function will be called when the button was held down for 1 second or more.
void pressStart() {
  //Serial.println("pressStart()");
  //pressStartTime = millis() - 1000; // as set in setPressTicks()
} // pressStart()


// this function will be called when the button was released after a long hold.
void pressStop() {
  //Serial.print("pressStop(");
  //Serial.print(millis() - pressStartTime);
  //Serial.println(") detected.");
} // pressStop()
void setup()  // Start of setup:
{

  Serial.begin(115200);  // Begin serial communication.
                         //wifi.setmode(wifi.NULLMODE);
  setModemSleep();
  
  pinMode(23, OUTPUT);// notskip
  pinMode(26, OUTPUT); // done
  digitalWrite(23, 1);digitalWrite(26, 0);
  rotaryEncoder.begin();
  rotaryEncoder.setup(readEncoderISR);
  rotaryEncoder.setBoundaries(-999, 999, false);  //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
  rotaryEncoder.setAcceleration(100);

  attachInterrupt(digitalPinToInterrupt(18), Pulse_Event_Light_Sensor, RISING);  // Enable interruption pin 3 when going from LOW to HIGH.
  attachInterrupt(digitalPinToInterrupt(19), Pulse_Event_Mic, RISING);           // Enable interruption pin 2 when going from LOW to HIGH.

 // setup interrupt routine
  // when not registering to the interrupt the sketch also works when the tick is called frequently.
  attachInterrupt(digitalPinToInterrupt(16), checkTicks, CHANGE);


  // To prevent from starting micros on sensors on 0, set as if they are already on limit
  Micros_For_Light_Sensor = 0 - TimeOutAmount;
  Micros_For_Mic = 0 - TimeOutAmount;
  //delay(500);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Initialize display with the I2C address of 0x3C (0X78)

  display.clearDisplay();  // Clear the buffer

  display.setTextColor(WHITE);  // Set color of the text

  display.setRotation(0);  // Set orientation. Goes from 0, 1, 2 or 3

  display.setTextWrap(false);  // By default, long lines of text are set to automatically “wrap” back to the leftmost column.
                               // To override this behavior (so text will run off the right side of the display - useful for
                               // scrolling marquee effects), use setTextWrap(false). The normal wrapping behavior is restored
                               // with setTextWrap
  
  
  // link the xxxclick functions to be called on xxxclick event.
  button.attachClick(singleClick);
  button.attachDoubleClick(doubleClick);
  button.attachMultiClick(multiClick);

  button.setPressTicks(1000); // that is the time when LongPressStart is called
  button.attachLongPressStart(pressStart);
  button.attachLongPressStop(pressStop);

}  // End of setup ******************************************


void rotary_onButtonClick() {
  static unsigned long lastTimePressed = 0;
  //ignore multiple press in that time milliseconds
  if (millis() - lastTimePressed < 500) {
    return;
  }
  lastTimePressed = millis();
  done = false;
  delay(300);
  }


void rotary_loop() {
  //dont print anything unless value changed
  if (rotaryEncoder.encoderChanged()) {
    //Serial.print("Value: ");
    Offset = rotaryEncoder.readEncoder();
    
    //Serial.println(rotaryEncoder.readEncoder());
  }
  if (rotaryEncoder.isEncoderButtonClicked()) {
    rotary_onButtonClick();
  }
}



  



void loop()  // Start of loop:
{
  delay(10);
  rotary_loop();
  
  button.tick();
  display123();
  //delay(10);
  
}



void display123() {
  if (notskip)   {
       if (done) { return; }
        }
  

  CurrentMicros = micros();  // Store the current micros to see if wait long enough

  // Check if enough time happen after a pulse to see if it is time to be ready for next test:
  if (CurrentMicros > Micros_For_Light_Sensor + TimeOutAmount) {
    Ready_Light = 1;  // Store that we are ready for a new test
  }


  // Check if enough time happen after a pulse to see if it is time to be ready for next test:
  if (CurrentMicros > Micros_For_Mic + TimeOutAmount) {
    Ready_Mic = 1;  // Store that we are ready for a new test
  }

  // Check if we are ready for a new test:
  if (Ready_Light == 1 && Ready_Mic == 1)  // If both sensors are ready
  {
    Ready_State = 1;  // Store that we are ready for a new test
    FirstLight = 0;   // Reset who is first
    FirstMic = 0;     // Reset who is first
    WeAreWaiting = 0;
  } else {
    Ready_State = 0;  // Store that we are not ready for a new test
  }


  if (Ready_Light == 0 && Ready_Mic == 0)  // If both sensors are triggered
  {
    WeAreWaiting = 0;  // Record that we are not waiting for a pulse
  }


  // Claculate difference between both pulses:
  Difference = Micros_For_Mic - Micros_For_Light_Sensor + (Offset * 1000);

  // Convert into milliseconds:
  Latency = Difference / 1000;



  // Convert Latency into a string, so we can change the text alignment to the right:
  // It can be also used to add or remove decimal numbers.
  char string[10];  // Create a character array of 10 characters
  // Convert float to a string:
  dtostrf(Latency, 4, 0, string);  // (<variable>,<amount of digits we are going to use>,<amount of decimal digits>,<string name>)



  /////////// Display:

  display.clearDisplay();  // Clear the display so we can refresh



  if (Ready_State == 1)  // If we are ready for a new test
  {
    // Print text:

    display.setFont(&FreeMono12pt7b);  // Set a custom font

    display.setCursor(10, 40);  // (x,y)
    display.println("NAV_SYNC");

    display.setCursor(10, 60);  // (x,y)

    display.println("Ready...");
    display.setCursor(30,10);
    display.setFont();
    display.print("Offset = ");
    display.print(Offset);


    // Text or value to print
  } else  // If we are in other state other than ready
  {
    // Print who is first:
    if (FirstLight == 1) {
      // Print text:
      display.setFont(&FreeMono9pt7b);  // Set a custom font
      display.setCursor(0, 20);         // (x,y)

      display.println("Audio Late");
      ShowOffset();  // Text or value to print
    }

    if (FirstMic == 1) {
      // Print text:
      display.setFont(&FreeMono9pt7b);  // Set a custom font
      display.setCursor(0, 20);         // (x,y)

      display.println("Audio Early");
      ShowOffset();  // Text or value to print
    }                // End of print who is first



    // Print "waiting for the other pulse":
    if (Ready_Light == 0 && Ready_Mic == 1 && WeAreWaiting == 1)  // If we had a Light pulse but not Mic yet
    {
      display.setFont();         // Set a custom font
      display.setCursor(0, 40);  // (x,y)

      display.println("Waiting for Audio");
      ShowOffset();  // Text or value to print
    }
    if (Ready_Light == 1 && Ready_Mic == 0 && WeAreWaiting == 1)  // If we had a Mic pulse but not Light yet
    {
      display.setFont();         // Set a custom font
      display.setCursor(10, 40);  // (x,y)

      display.println("Waiting for Video");
      ShowOffset();  // Text or value to print
    }


    // If we had both pulses, we are going to print latency:
    if (Ready_Light == 0 && Ready_Mic == 0 || WeAreWaiting == 0)  // If we had both pulses
    {
      // Print latency value:
      ShowOffset();
      display.setFont(&FreeMonoBold18pt7b);  // Set a custom font
      display.setCursor(5, 63);              // (x,y)

      display.println(string);  // Text or value to print

      // Print text (ms):
      display.setFont(&FreeMono12pt7b);  // Set a custom font
      display.setCursor(95, 35);         // (x,y)

      display.println("ms");
      if (notskip) {
          done = true; 
          digitalWrite(26, 1);
       } else {
         done = false; 
         digitalWrite(26, 0);}
        
    }
  }   

  display.display();  // Print everything we set previously


  //////////////////////////////////////////////////////////////////////
}

void ShowOffset() {
  display.setFont();  // Set a custom font
  display.setCursor(0, 28);
  display.print("Offset = ");
  display.println(Offset);
}

void Pulse_Event_Light_Sensor()  // The interrupt runs this:
{
  
  // Check if we are in ready state or waiting for pulse:
  if (Ready_State == 1 || WeAreWaiting == 1) {

    if (Ready_Light == 1)  // If we are in ready state
    {
      Micros_For_Light_Sensor = micros();  // Stores the current micros so the next time we have a pulse we would have something to compare with.
      Ready_Light = 0;                     // Remove the "Ready" state so we don't take another pulse

      WeAreWaiting = 1;

      if (FirstMic == 0)  // If Mic haven't had a pulse yet
      {
        FirstLight = 1;  // Record that Light was first
      }
    }
  }
}  // End of Pulse_Event_Light_Sensor





void Pulse_Event_Mic()  // The interrupt runs this
{
  
  // Check if we are in ready state or waiting for pulse:
  if (Ready_State == 1 || WeAreWaiting == 1) {


    if (Ready_Mic == 1)  // If we are in ready state
    {
      Micros_For_Mic = micros();  // Stores the current micros so the next time we have a pulse we would have something to compare with.
      Ready_Mic = 0;              // Remove the "Ready" state so we don't take another pulse

      WeAreWaiting = 1;

      if (FirstLight == 0)  // If Light haven't had a pulse yet
      {
        FirstMic = 1;  // Record that Mic was first
      }
    }
  }
}  // End of Pulse_Event_Mic

void disableWiFi() {
  //adc_power_off();
  WiFi.disconnect(true);  // Disconnect from the network
  WiFi.mode(WIFI_OFF);    // Switch WiFi off
  Serial.println("");
  Serial.println("WiFi disconnected!");
}
void disableBluetooth() {
  // Quite unusefully, no relevable power consumption
  btStop();
  Serial.println("");
  Serial.println("Bluetooth stop!");
}

void setModemSleep() {
  disableWiFi();
  disableBluetooth();
  //setCpuFrequencyMhz(40);
  // Use this if 40Mhz is not supported
  // setCpuFrequencyMhz(80);
}