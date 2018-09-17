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

**e. What kind of relationship does the resistance have as a function of stimulus? (e.g., linear?)**

### 2. Accelerometer
 
**a. Include your accelerometer read-out code in your write-up.**

### 3. IR Proximity Sensor

**a. Describe the voltage change over the sensing range of the sensor. A sketch of voltage vs. distance would work also. Does it match up with what you expect from the datasheet?**

**b. Upload your merged code to your lab report repository and link to it here.**

## Optional. Graphic Display

**Take a picture of your screen working insert it here!**

## Part D. Logging values to the EEPROM and reading them back
 
### 1. Reading and writing values to the Arduino EEPROM

**a. Does it matter what actions are assigned to which state? Why?**

**b. Why is the code here all in the setup() functions and not in the loop() functions?**

**c. How many byte-sized data samples can you store on the Atmega328?**

**d. How would you get analog data from the Arduino analog pins to be byte-sized? How about analog data from the I2C devices?**

**e. Alternately, how would we store the data if it were bigger than a byte? (hint: take a look at the [EEPROMPut](https://www.arduino.cc/en/Reference/EEPROMPut) example)**

**Upload your modified code that takes in analog values from your sensors and prints them back out to the Arduino Serial Monitor.**

### 2. Design your logger
 
**a. Insert here a copy of your final state diagram.**

### 3. Create your data logger!
 
**a. Record and upload a short demo video of your logger in action.**
