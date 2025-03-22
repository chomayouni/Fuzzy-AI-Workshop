#include <SimpleUltrasonic.h>
#include <Fuzzy.h>

#define PWM_D9_PIN 9  // Must be 9 or 10 (Timer1-based PWM)
#define JOYSTICK_PIN A0


#define PRINTS

// Ultrasonic Sensor (Trigger, Echo)
SimpleUltrasonic sensor(3, 2);

float currentFanPWM = 90.0; // Start with medium value

// Singleton Outputs:
// #define MAX_DELTA 0.015
// #define MIN_DELTA 0.01
//const float VERY_LOW_SPEED = (-MIN_DELTA);
//const float LOW_SPEED = (-MIN_DELTA/2);
//const float MID_SPEED =  0.0;
//const float HIGH_SPEED = (MAX_DELTA/2);
//const float VERY_HIGH_SPEED = (MAX_DELTA);

const float VERY_LOW_SPEED = -0.01;
const float LOW_SPEED = -0.005;
const float MID_SPEED = 0.0;
const float HIGH_SPEED = 0.005;
const float VERY_HIGH_SPEED = 0.01;

// Fuzzy Logic System
Fuzzy *fuzzy = new Fuzzy();
FuzzySet *neg_far, *neg_close, *zero_p, *pos_close, *pos_far;  // Target error
FuzzySet *neg_high, *neg_low, *zero_m, *pos_low, *pos_high;
FuzzySet *neg_fast, *neg_slow, *zero_s, *pos_slow, *pos_fast; // Speeds

// Toggle mode: 0 = Joystick (Manual), 1 = Fuzzy (Automatic)
bool fuzzyMode = true;

// Target position (in inches)
//const float targetDistance = 9.0;  // Set your desired target distance here

// Moving Average Filter for Smoothing Distance Measurements
float smoothedDistance = 0;
const int smoothingFactor = 10;  // Number of samples to average
float previousDistance = 0;
unsigned long previousTime, timea, timeb = 0;
float ballSpeed = 0;
float targetError = 0;  // Difference from target position
float currentTargetPosition = 12.0;  // Start at middle of range
const float minTargetPosition = 3.0;
const float maxTargetPosition = 28.0;

void updateTargetWithJoystick() {
    // Read joystick value
    int joystickValue = analogRead(JOYSTICK_PIN);
    
    // Calculate rate of change based on joystick position
    // Center position (around 512) = no change
    // Full right/up = fast increase, full left/down = fast decrease
    float rate = mapfloat(joystickValue, 0.0, 1023.0, -0.005, 0.005);
    
    // Create a dead zone in the middle to prevent drift
    if (abs(rate) < 0.002) {
        rate = 0.0;
    }
    
    // Update target position at constant rate
    currentTargetPosition += rate;
    
    // Constrain to valid range
    currentTargetPosition = constrain(currentTargetPosition, minTargetPosition, maxTargetPosition);
}


float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  // Check if input is out of range and constrain it
  if (x < in_min) x = in_min;
  if (x > in_max) x = in_max;
  
  // Perform the mapping calculation
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void updateSmoothedDistance(float newReading) {
    static float readings[smoothingFactor] = {0};
    static int index = 0;

    // Reject clearly erroneous readings
    if (newReading > 0 && newReading < 32) {  // Valid range check
        readings[index] = newReading;
        index = (index + 1) % smoothingFactor;

        // Use weighted moving average with emphasis on recent readings
        float sum = 0;
        float weightSum = 0;
        for (int i = 0; i < smoothingFactor; i++) {
            int j = (index - 1 - i + smoothingFactor) % smoothingFactor;
            float weight = smoothingFactor - i;
            if (readings[j] > 0) {  // Only include valid readings
                sum += readings[j] * weight;
                weightSum += weight;
            }
        }
        
        if (weightSum > 0) {
            smoothedDistance = sum / weightSum;
        }
    }
}

void updateBallSpeed() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0;
    
    if (deltaTime >= 0.05) {
        float instantSpeed = (previousDistance - smoothedDistance) / deltaTime;
        
        // Stronger filtering for better stability
        const float speedFilterAlpha = 0.15; // Lower value for more filtering
        ballSpeed = (speedFilterAlpha * instantSpeed) + ((1.0 - speedFilterAlpha) * ballSpeed);
        
        // Larger dead zone for near-zero speeds
        if (abs(ballSpeed) < 0.15) {
            ballSpeed = 0.0;
        }
        
        // Lower max speed values for more gentle transitions
        ballSpeed = constrain(ballSpeed, -8.0, 8.0);
        
        previousDistance = smoothedDistance;
        previousTime = currentTime;
    }
}


void setupFuzzy() {
    // Define input fuzzy variable for target error
    FuzzyInput *errorInput = new FuzzyInput(1);
    neg_far = new FuzzySet(0, 0, 40, 85);      // Far below target
    neg_close = new FuzzySet(65, 90, 110, 125);    // Below target
    zero_p = new FuzzySet(100, 115, 130, 140);        // At target
    pos_close = new FuzzySet(130, 140, 155, 175);        // Above target  
    pos_far = new FuzzySet(150, 170, 255, 255);          // Far above target
    errorInput->addFuzzySet(neg_far);
    errorInput->addFuzzySet(neg_close);
    errorInput->addFuzzySet(zero_p);
    errorInput->addFuzzySet(pos_close);
    errorInput->addFuzzySet(pos_far);
    fuzzy->addFuzzyInput(errorInput);

    FuzzyInput *speedInput = new FuzzyInput(2);
    neg_fast = new FuzzySet(0, 0, 20, 90);
    neg_slow = new FuzzySet(50, 70, 90, 120);   
    zero_s = new FuzzySet(100, 125, 135, 155);    
    pos_slow = new FuzzySet(140, 170, 180, 200);     
    pos_fast = new FuzzySet(150, 200, 255, 255);    
    speedInput->addFuzzySet(neg_fast);
    speedInput->addFuzzySet(neg_slow);
    speedInput->addFuzzySet(zero_s);
    speedInput->addFuzzySet(pos_slow);
    speedInput->addFuzzySet(pos_fast);
    fuzzy->addFuzzyInput(speedInput);

    FuzzyOutput *fanSpeed = new FuzzyOutput(1);
    neg_high = new FuzzySet(VERY_LOW_SPEED, VERY_LOW_SPEED, VERY_LOW_SPEED, VERY_LOW_SPEED);
    neg_low = new FuzzySet(LOW_SPEED, LOW_SPEED, LOW_SPEED, LOW_SPEED);      
    zero_m = new FuzzySet(MID_SPEED, MID_SPEED, MID_SPEED, MID_SPEED);   
    pos_low = new FuzzySet(HIGH_SPEED, HIGH_SPEED, HIGH_SPEED, HIGH_SPEED);     
    pos_high = new FuzzySet(VERY_HIGH_SPEED, VERY_HIGH_SPEED, VERY_HIGH_SPEED, VERY_HIGH_SPEED); 
    fanSpeed->addFuzzySet(neg_high);
    fanSpeed->addFuzzySet(neg_low);
    fanSpeed->addFuzzySet(zero_m);
    fanSpeed->addFuzzySet(pos_low);
    fanSpeed->addFuzzySet(pos_high);
    fuzzy->addFuzzyOutput(fanSpeed);

    // Generate fuzzy rules from K-map
    int ruleIndex = 1;
    FuzzySet *speedLevels[] = {pos_fast, pos_slow, zero_s, neg_slow, neg_fast};
    FuzzySet *posLevels[] = {neg_far, neg_close, zero_p, pos_close, pos_far};
    FuzzySet *outputLevels[5][5] = {
        {NULL, neg_low, neg_high, neg_high, NULL},
        {pos_low, zero_m, neg_low, neg_high, neg_high},
        {pos_high, pos_low, zero_m, neg_low, neg_high},
        {pos_high, pos_high, pos_low, zero_m, neg_low},
        {NULL, pos_high, pos_high, pos_low, NULL}
    };
   
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
          if (outputLevels[i][j] != NULL) 
          {
            FuzzyRuleAntecedent *antecedent = new FuzzyRuleAntecedent();
            antecedent->joinWithAND(speedLevels[i], posLevels[j]);
            FuzzyRuleConsequent *consequent = new FuzzyRuleConsequent();
            consequent->addOutput(outputLevels[i][j]);
            fuzzy->addFuzzyRule(new FuzzyRule(ruleIndex++, antecedent, consequent));
          }
        }
    }
}

void setupPWM() {
    pinMode(PWM_D9_PIN, OUTPUT);
    TCCR1A = _BV(COM1A1) | _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
    ICR1 = 320;  // 25kHz frequency
}

void setFanSpeed(float percentage) {
    OCR1A = (percentage / 100.0) * 320;  // Convert 0-100% to Timer1 PWM value
}

void setup() {
    Serial.begin(115200);
    sensor.begin();
    setupFuzzy();
    setupPWM();
    previousTime = millis();
}

void loop() {
    timea = millis();
    float distance = sensor.measureDistanceInInch();
    updateSmoothedDistance(distance);
    if (fuzzyMode) {
        updateBallSpeed();
        
        // Update target at constant rate based on joystick
        updateTargetWithJoystick();
        
        // Calculate error using the current target
        targetError = currentTargetPosition - smoothedDistance;
        targetError = constrain(targetError, -6.0, 6.0);
        float mappederror = mapfloat(targetError, -6.0, 6.0, 0.0, 255.0);
        ballSpeed = constrain(ballSpeed, -6.0, 6.0);
        float mappedspeed = mapfloat(ballSpeed, -6.0, 6.0, 0.0, 255.0);
        
        if (distance != -1) {
            #ifdef PRINTS
                Serial.print("Distance: ");
                Serial.print(smoothedDistance);
                Serial.print(" in\tTarget: ");
                Serial.print(currentTargetPosition);  // Print the current target
                Serial.print(" in\tError: ");
                Serial.print(targetError);
                Serial.print(" in\tSpeed: ");
                Serial.print(ballSpeed);
                Serial.print(" in/sec");
            #endif
            
            fuzzy->setInput(1, mappederror);
            fuzzy->setInput(2, mappedspeed);
            fuzzy->fuzzify();
            float fanPWM_delta = fuzzy->defuzzify(1);
            
            // // Adaptive rate limiting - faster during large errors
            // float maxChange = 0.2; // Normal rate limit
            // if (abs(targetError) > 3.0) {
            //     maxChange = 0.2; // Faster response for large errors
            // }
            
            // // Apply rate limiting
            // if (fanPWM_delta > lastfanPWM_delta + maxChange) {
            //     fanPWM_delta = lastfanPWM_delta + maxChange;
            // } else if (fanPWM_delta < lastfanPWM_delta - maxChange) {
            //     fanPWM_delta = lastfanPWM_delta - maxChange;
            // }
            
            currentFanPWM += fanPWM_delta;
            currentFanPWM = constrain(currentFanPWM, 0.0, 100.0);
            
            #ifdef PRINTS
                Serial.print("\tDelta speed: ");
                Serial.print(fanPWM_delta);
                Serial.print("\tFan speed: ");
                Serial.println(currentFanPWM);
            #endif

            // Apply fan power
            setFanSpeed(currentFanPWM);
        } else {
            Serial.println("Error: Measurement timeout");
        }
    } else {
        int joystickValue = analogRead(JOYSTICK_PIN);
        float pwmDutyCycle = mapfloat(joystickValue, 0.0, 1023.0, 50.0, 100.0);
        setFanSpeed(pwmDutyCycle);
        #ifdef PRINTS
            Serial.print("Manual Mode: \t Distance: ");
            Serial.print(smoothedDistance);
            Serial.print("\tFanspeed(%): ");
            Serial.println(pwmDutyCycle);
        #endif
    }
           // Debug output for input and output fuzzy sets
        // Serial.println("Error membership values:");
        // Serial.print("veryNegativeError: ");
        // Serial.println(veryNegativeError->getPertinence());
        // Serial.print("negativeError: ");
        // Serial.println(negativeError->getPertinence());
        // Serial.print("zeroError: ");
        // Serial.println(zeroError->getPertinence());
        // Serial.print("positiveError: ");
        // Serial.println(positiveError->getPertinence());
        // Serial.print("veryPositiveError: ");
        // Serial.println(veryPositiveError->getPertinence());
        
        // Serial.println("Speed membership values:");
        // Serial.print("goingDownFast: ");
        // Serial.println(goingDownFast->getPertinence());
        // Serial.print("goingDown: ");
        // Serial.println(goingDown->getPertinence());
        // Serial.print("steady: ");
        // Serial.println(steady->getPertinence());
        // Serial.print("goingUp: ");
        // Serial.println(goingUp->getPertinence());
        // Serial.print("goingUpFast: ");
        // Serial.println(goingUpFast->getPertinence());
         
        // Debug output for rule activations
        // Serial.println("Rule activations:");
        // bool anyRuleActive = false;
        // for (int i = 1; i <= 25; i++) {  // Now using 20 rules
        //     float activation = fuzzy->isFiredRule(i);
        //     // if (activation > 0) {
        //         anyRuleActive = true;
        //         Serial.print("Rule ");
        //         Serial.print(i);
        //         Serial.print(": ");
        //         Serial.print(activation);
        //         Serial.print(" ");
        //     // }
        // }
        // Serial.println(" ");
        
        // if (!anyRuleActive) {
        //     Serial.println("WARNING: No rules are active!");
        // }

    timeb = millis();
    delay(5);
}
