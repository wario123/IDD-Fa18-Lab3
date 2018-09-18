# Data Logger (and using cool sensors!)

*A lab report by Karim Arem.*


## Part A.  Writing to the Serial Monitor
 
**a. Based on the readings from the serial monitor, what is the range of the analog values being read?**

The range is from 0 to 1023.
 
**b. How many bits of resolution does the analog to digital converter (ADC) on the Arduino have?**

The arduino 10-bit analog to digital converter.

## Part B. RGB LED

**How might you use this with only the parts in your kit? Show us your solution.**

## Part C. Voltage Varying Sensors 
 
### 1. FSR, Flex Sensor, Photo cell, Softpot

**a. What voltage values do you see from your force sensor?**

The voltage values range from 0 to 5V. When seen on the Serial Monitor, the values from 0 to 5 are mapped to 10 bit numbers (0 to 1023)

**b. What kind of relationship does the voltage have as a function of the force applied? (e.g., linear?)**

It seems that as the voltage has a logarithmic relationship with the force applied because by applying only a little pressure the values on the serial monitor jump immediately to 700-800 and applying a large amount of force increases the voltage values by a small amount.

**c. Can you change the LED fading code values so that you get the full range of output voltages from the LED when using your FSR?**

We can combine the rgb LED code with the fade code and connect the fsr to input into the color setting function.

```
int redPin = 8;
int greenPin = 10;
int bluePin = 9;         // the PWM pin the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by
int fsrAnalogPin = A5;
int force;
int x;
int y;
int z;

// the setup routine runs once when you press reset:
void setup() {
  // declare pin 9 to be an output:
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // set the brightness of pin 9:
  force = analogRead(fsrAnalogPin);
  x = (force-70) % 255;
  y = (force-120) % 255;
  z = force % 255;
  
  setColor(x, y, z);

//  // change the brightness for next time through the loop:
//  brightness = brightness + fadeAmount;
//
//  // reverse the direction of the fading at the ends of the fade:
//  if (brightness <= 0 || brightness >= 255) {
//    fadeAmount = -fadeAmount;
//  }
  // wait for 30 milliseconds to see the dimming effect
  delay(30);
}

void setColor(int red, int green, int blue)
{
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}

```

**d. What resistance do you need to have in series to get a reasonable range of voltages from each sensor?**

Photo Cell: 22K Ohm
SoftPot:
Flex Sensor:

**e. What kind of relationship does the resistance have as a function of stimulus? (e.g., linear?)**

Flex: Logarithmic
Photo Cell: Linear
SoftPt: Linear

### 2. Accelerometer
 
**a. Include your accelerometer read-out code in your write-up.**

```
// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// include the library code for LCD:
#include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int analogPin = 0;

int redPin = 8;
int greenPin = 10;
int bluePin = 9;


//// Used for software SPI
//#define LIS3DH_CLK 13
//#define LIS3DH_MISO 12
//#define LIS3DH_MOSI 11
//// Used for hardware & software SPI
//#define LIS3DH_CS 10

// software SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif

void setup(void) {
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif

  Serial.begin(9600);
  Serial.println("LIS3DH test!");
  
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("LIS3DH found!");
  
  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
  
  Serial.print("Range = "); Serial.print(2 << lis.getRange());  
  Serial.println("G");

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);  
}

void loop() {
  lis.read();      // get X Y and Z data at once
  // Then print out the raw data
  Serial.print("X:  "); Serial.print(lis.x); 
  Serial.print("  \tY:  "); Serial.print(lis.y); 
  Serial.print("  \tZ:  "); Serial.print(lis.z); 

  /* Or....get a new sensor event, normalized */ 
  sensors_event_t event; 
  lis.getEvent(&event);
  
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
  Serial.print(" \tY: "); Serial.print(event.acceleration.y); 
  Serial.print(" \tZ: "); Serial.print(event.acceleration.z); 
  Serial.println(" m/s^2 ");


  

  Serial.println();
  lcd.clear();
  lcd.print("X:"); lcd.print(event.acceleration.x);
  lcd.print(" Y:"); lcd.print(event.acceleration.y); lcd.setCursor(0,1); 
  lcd.print(" Z:"); lcd.print(event.acceleration.z); 
  setColor(event.acceleration.x*25, event.acceleration.y*25, event.acceleration.z*25);
  delay(200); 
}

void setColor(float red, float  green, float blue)
{
  #ifdef COMMON_ANODE
    red = 255 - red;
    green = 255 - green;
    blue = 255 - blue;
  #endif
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);  
}
```

### 3. IR Proximity Sensor

**a. Describe the voltage change over the sensing range of the sensor. A sketch of voltage vs. distance would work also. Does it match up with what you expect from the datasheet?**

![alt tag](https://github.com/wario123/IDD-Fa18-Lab3/blob/master/Screen%20Shot%202018-09-17%20at%205.49.09%20PM.png)

From the graph above, as we linearly approach our finger to the proximity sensor the voltage values exponentially increase. 60,000 corresponds to touching the proximity sensor, 30,000 corresponds to around 4 mm, 10,000 corresponds to around 1.5 cm while 3,000 corresponds to aroun 5 cm.

**b. Upload your merged code to your lab report repository and link to it here.**

[Link to Code!](https://github.com/wario123/IDD-Fa18-Lab3/blob/master/accel_and_prox.ino)

## Optional. Graphic Display

**Take a picture of your screen working insert it here!**

## Part D. Logging values to the EEPROM and reading them back
 
### 1. Reading and writing values to the Arduino EEPROM

**a. Does it matter what actions are assigned to which state? Why?**

It does because if the writing state is when the potentiometer is set to 1 and  the clearing state is when potentiometer is in the middle (set to 2) then you will always clear what your write if you want to try to read a value and thus will never be able to read a value that is in memeory.

**b. Why is the code here all in the setup() functions and not in the loop() functions?**

It is all in the setup() functions because we do not want to perform the state actions indefinitely. We only want to write once or read once to memory.

**c. How many byte-sized data samples can you store on the Atmega328?**

We can store 1024 byte sized data samples on the Atmega328.

**d. How would you get analog data from the Arduino analog pins to be byte-sized? How about analog data from the I2C devices?**



**e. Alternately, how would we store the data if it were bigger than a byte? (hint: take a look at the [EEPROMPut](https://www.arduino.cc/en/Reference/EEPROMPut) example)**

We could store the rest of the data that does not fit into the next memory location.

**Upload your modified code that takes in analog values from your sensors and prints them back out to the Arduino Serial Monitor.**

### 2. Design your logger
 
**a. Insert here a copy of your final state diagram.**

### 3. Create your data logger!
 
**a. Record and upload a short demo video of your logger in action.**
