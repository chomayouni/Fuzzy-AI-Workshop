# Floating Ball Arduino Project: Using Fuzzy Logic



## Introduction

Welcome to the Floating Ball project! In this experiment, we'll use an Arduino to make a ball float in the air at a specific height. 

This project uses something called "fuzzy logic," which is a type of artificial intelligence that helps computers make decisions in a way that's more like how humans think. Instead of just "on" or "off," fuzzy logic can handle "kind of" or "almost" or "a little bit" - just like we do when we're thinking!

By the end of this project, you'll be able to control a floating ball using a joystick or let the Arduino control it automatically with fuzzy logic. 

![image](https://github.com/user-attachments/assets/53250bc2-0b13-4796-923f-bc8e90da0316) ![image](https://github.com/user-attachments/assets/22510e72-02c6-4eee-9231-7c15ab852d44)



## What You'll Need (Parts List)

![image](https://github.com/user-attachments/assets/8156e95c-4fef-4879-8915-150d8aa9c319)


- Arduino Uno or compatible board
- Ultrasonic distance sensor (HC-SR04)
- Small DC fan (5V or 12V) - We uses a 12V 80mm computer fan
- MOSFET transistor (for controlling the fan speed)
- Joystick module
- Ping pong ball (or other lightweight ball)
- Clear tube (where the ball will float)
- Breadboard and jumper wires
- USB cable for Arduino
- Power supply for the fan (depending on your fan's requirements)

## Library Installation

Before we start coding, we need to install some special libraries for our Arduino. Libraries are like extra tools that make it easier for us to use certain components.

### How to Install Libraries:

1. Open the Arduino IDE (the program we use to write code for Arduino)
2. Click on "Sketch" in the top menu
3. Select "Include Library" and then "Manage Libraries..."
4. In the search box, type each library name and click "Install" for each one:
   - "SimpleUltrasonic" - For measuring distance with the ultrasonic sensor
   - "Fuzzy" - For implementing fuzzy logic control

If you can't find these libraries, you can download them from these links:
- SimpleUltrasonic: https://github.com/arduino-libraries/SimpleUltrasonic
- Fuzzy: https://github.com/zerokol/eFLL (This is the eFLL - Embedded Fuzzy Logic Library)

## Hardware Assembly

Let's put everything together! Here's a step-by-step guide:

TODO: put circuit diagram here

1. **Assemble the fan housing**: Grab the M3 Bolts, M3 nuts, the 80mm fan, and the 3D printed housings for the top and the bottom of the fans
   
   ![image](https://github.com/user-attachments/assets/68304930-245b-4a2a-a0c4-11d2b2acb600)
   
   - Sandwch the fan between the 2 housings and insert the two M3 bolts as shown below:
     
     ![image](https://github.com/user-attachments/assets/859facb2-05ab-41ac-a505-3a1fa8853771)

   - Turn the module on its side and gently screw on the nuts. Be careful to nobreak the legs attached to the intake of the fan.
     
     ![image](https://github.com/user-attachments/assets/4cd06f3c-8fad-4c84-9695-e35cea70157c)

3. **Set up the tube**: Place your clear tube vertically in the 3D printed fan adapter - It should friction fit right in.
   
      ![image](https://github.com/user-attachments/assets/dfc8c834-01e7-4533-a51d-d2ab74186d78)

4.  **Assemble the Ultrasonic Sensor housing**:
   - Take the bracket and thread a ziptie thorugh
     
     ![image](https://github.com/user-attachments/assets/d6c983f3-6e39-4501-8fba-afd0454b3e9f)

   - Attach the bracket to the top of the tub. It should be tight enough to stay, but loose enough to adjust
     
     ![image](https://github.com/user-attachments/assets/11dd11c1-3586-4b91-9dba-130077e78574)

   - Insert the Ultrasonic sensor facing down into the tube. Secure it with electrical tape
     
     ![image](https://github.com/user-attachments/assets/c067db87-8590-4795-9d83-71c52a7d83d8)


6.  **Connect the ultrasonic sensor**:
   - Connect VCC pin to 5V on Arduino
   - Connect GND pin to GND on Arduino
   - Connect Trigger pin to Digital pin 2 on Arduino
   - Connect Echo pin to Digital pin 3 on Arduino

6. **Connect the joystick**:
   - Connect VCC to 5V on Arduino
   - Connect GND to GND on Arduino
   - Connect the X or Y output to Analog pin A0 on Arduino

7. **Connect the fan with MOSFET**:
   - Connect the fan's power wire to your power supply positive terminal
   - Connect the fan's ground wire to the MOSFET's drain pin
   - Connect the MOSFET's source pin to the power supply ground
   - Connect the MOSFET's gate pin to Digital pin 9 on Arduino
   - Add a 10K resistor between the MOSFET's gate and ground

8. **Final check**:
   - Make sure all ground connections are connected together
   - Double-check all connections before powering up
   - Place the fan at the bottom of the tube, pointing upward
  
   ![image](https://github.com/user-attachments/assets/12f245a0-be7b-4bc7-b3c2-7e8282d829f6)


## Understanding the Code

Our code does several important things:

1. **Reads the distance** to the ball using the ultrasonic sensor
2. **Smooths the readings** to prevent jittery measurements
3. **Calculates the ball's speed** to help make better decisions
4. **Uses fuzzy logic** to decide how fast the fan should blow
5. **Controls the fan speed** to keep the ball at the target position
6. **Reads the joystick** to let you change the target position

Let's break down some of the most important parts:

### Measuring Distance

The code uses the ultrasonic sensor to measure how far away the ball is. It sends out sound waves that bounce off the ball and come back. By measuring how long this takes, we can figure out the distance.

### Fuzzy Logic

This is the AI part! Fuzzy logic helps the Arduino decide how to control the fan based on:
- How far the ball is from the target position (the "error")
- How fast the ball is moving (going up, going down, or steady)

The code sets up rules like:
- If the ball is far below the target AND moving downward, turn the fan on high
- If the ball is at the target AND not moving much, keep the fan at medium
- If the ball is above the target AND moving upward, turn the fan to very low

### PWM Fan Control

PWM stands for "Pulse Width Modulation," and it's how we control the fan speed. Instead of just turning the fan on or off, PWM rapidly switches the power on and off. The more time it's on versus off, the faster the fan spins.

## How to Use the Project

1. **Upload the code** to your Arduino using the Arduino IDE
2. **Position everything**:
   - Fan at the bottom of the tube
   - Ball inside the tube
   - Ultrasonic sensor at the top of the tube, pointing down
3. **Power on** your Arduino and the fan power supply
4. **Watch the ball float**!
5. **Use the joystick** to change the target height of the ball

The default mode is "Fuzzy Mode" which means the Arduino's fuzzy logic is controlling the fan to keep the ball at the target height. You can move the joystick to change the target height.

## Introduction to Fuzzy Logic

Fuzzy logic is really cool because it works more like human thinking than regular computer logic. Here's a simple explanation:

### Regular (Boolean) Logic vs. Fuzzy Logic:

- **Regular Logic**: Things are either TRUE (1) or FALSE (0). A light is either ON or OFF.
- **Fuzzy Logic**: Things can be partially true. A light can be DIM, BRIGHT, or anything in between.

![Fuzzy Logic Diagram](https://via.placeholder.com/800x400?text=Fuzzy+Logic+Explanation)

In our project, we use fuzzy logic to describe:

1. **How far from the target**:
   - VERY NEGATIVE (way below target)
   - NEGATIVE (a bit below target)
   - ZERO (at target)
   - POSITIVE (a bit above target)
   - VERY POSITIVE (way above target)

2. **Ball speed**:
   - GOING DOWN (moving downward)
   - STEADY (not moving much)
   - GOING UP (moving upward)

3. **Fan speed**:
   - VERY LOW (75% power)
   - LOW (78% power)
   - MEDIUM (80% power)
   - HIGH (82% power)
   - VERY HIGH (85% power)

The fuzzy controller combines these to make decisions. For example, if the ball is NEGATIVE (below target) and GOING UP, the fuzzy logic might set the fan to LOW because the ball is already moving in the right direction.

## Tuning Your Project

If your ball isn't floating stably, you might need to tune some values in the code:

1. **Fan Power Levels**: Look for these lines in the code:
```cpp
veryLow = new FuzzySet(76.5, 76.5, 76.5, 76.5);
low = new FuzzySet(78.0, 78.0, 78.0, 78.0);
medium = new FuzzySet(79.5, 79.5, 79.5, 79.5);
high = new FuzzySet(81.0, 81.0, 81.0, 81.0);
veryHigh = new FuzzySet(83.0, 83.0, 83.0, 83.0);
```
   You can change these numbers to adjust the fan power levels. The values represent the percentage of full power.

2. **Target Position Range**: Look for these lines:
```cpp
const float minTargetPosition = 3.0;
const float maxTargetPosition = 12.0;
```
   These values set the minimum and maximum target heights (in inches). Adjust them based on the height of your tube.

3. **Speed Filter**: If the ball speed calculations seem too jumpy, you can adjust this line:
```cpp
const float speedFilterAlpha = 0.15;
```
   A lower value will make the speed changes smoother but less responsive.

## Troubleshooting

If you run into problems, here are some things to check:

1. **Ball won't float at all**: 
   - Make sure your fan is powerful enough for your ball
   - Check that the fan is receiving power
   - Verify that PWM pin 9 is correctly connected

2. **Ball is unstable (bouncing up and down)**:
   - Try adjusting the fuzzy logic values for a smoother response
   - Increase the smoothingFactor for distance readings
   - Check if there are any air currents in the room affecting the ball

3. **Readings are inconsistent**:
   - Make sure the ultrasonic sensor is positioned correctly
   - Check that the ball is within the detection range of the sensor
   - Try reducing outside noise that might affect the ultrasonic sensor

4. **Arduino not responding**:
   - Check USB connection
   - Verify that the correct port and board are selected in Arduino IDE
   - Make sure libraries are installed correctly

## Going Further

Once you get the basic project working, here are some cool ideas to try:

1. **Add an LED display** to show the current height and target height
2. **Add buttons** to switch between different preset heights
3. **Create a game** where you try to keep the ball within a certain zone
4. **Experiment with different fuzzy logic rules** to see how they affect performance
5. **Try different types of balls** to see how the system adapts

## How It Connects to Real-World AI

This project is a small example of how artificial intelligence can be used in the real world:

- **Self-driving cars** use fuzzy logic and other AI techniques to make decisions about speed, braking, and steering
- **Smart home thermostats** use similar techniques to maintain comfortable temperatures
- **Industrial machinery** often uses fuzzy logic controllers for precise positioning

Congratulations on building your own AI-controlled system! You've taken your first step into the exciting world of fuzzy logic and control systems.
