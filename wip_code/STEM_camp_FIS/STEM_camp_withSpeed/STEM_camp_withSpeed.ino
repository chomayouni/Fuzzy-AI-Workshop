#include <SimpleUltrasonic.h>
#include <Fuzzy.h>

#define PWM_D9_PIN 9  // Must be 9 or 10 (Timer1-based PWM)
#define JOYSTICK_PIN A0

#define PRINTS

// Ultrasonic Sensor (Trigger, Echo)
SimpleUltrasonic sensor(3, 2);

// Singleton Outputs:
#define MAX_SPEED 83
#define MIN_SPEED 76.5
const float VERY_LOW_SPEED (MIN_SPEED);
const float LOW_SPEED (MIN_SPEED + ((MAX_SPEED - MIN_SPEED) * 0.25));
const float MID_SPEED ((MAX_SPEED + MIN_SPEED) * 0.5);
const float HIGH_SPEED (MIN_SPEED + ((MAX_SPEED - MIN_SPEED) * 0.75));
const float VERY_HIGH_SPEED (MAX_SPEED);

// Fuzzy Logic System
Fuzzy *fuzzy = new Fuzzy();
FuzzySet *veryNegative, *negative, *zero, *positive, *veryPositive;  // Target error
FuzzySet *veryLow, *low, *mediumLow, *medium, *mediumHigh, *high, *veryHigh;
FuzzySet *goingDownFast, *goingDown, *steady, *goingUp, *goingUpFast;

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
float currentTargetPosition = 7.0;  // Start at middle of range
const float minTargetPosition = 3.0;
const float maxTargetPosition = 12.0;

void updateTargetWithJoystick() {
    // Read joystick value
    int joystickValue = analogRead(JOYSTICK_PIN);
    
    // Calculate rate of change based on joystick position
    // Center position (around 512) = no change
    // Full right/up = fast increase, full left/down = fast decrease
    float rate = mapfloat(joystickValue, 0.0, 1023.0, -0.05, 0.05);
    
    // Create a dead zone in the middle to prevent drift
    if (abs(rate) < 0.01) {
        rate = 0.0;
    }
    
    // Update target position at constant rate
    currentTargetPosition += rate;
    
    // Constrain to valid range
    currentTargetPosition = constrain(currentTargetPosition, minTargetPosition, maxTargetPosition);
}


float mapfloat(float in, float in_min, float in_max, float out_min, float out_max) {
    return (((in - in_min) / (in_max - in_min)) * (out_max - out_min) + out_min);
}

void updateSmoothedDistance(float newReading) {
    static float readings[smoothingFactor] = {0};
    static int index = 0;

    // Reject clearly erroneous readings
    if (newReading > 0 && newReading < 16) {  // Valid range check
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

//void updateTargetError() {
//    // Positive error means ball is above target (distance less than target)
//    // Negative error means ball is below target (distance more than target)
//    targetError = targetDistance - smoothedDistance;
//}

void setupFuzzy() {
    // Define input fuzzy variable for target error
    FuzzyInput *errorInput = new FuzzyInput(1);
    veryNegative = new FuzzySet(-7, -7, -5, -2);      // Far below target
    negative = new FuzzySet(-4, -2.5, -2.5, -0.5);    // Below target
    zero = new FuzzySet(-1.0, -0.2, 0.2, 1.0);        // At target
    positive = new FuzzySet(0.5, 2.5, 2.5, 4);        // Above target  
    veryPositive = new FuzzySet(2, 5, 7, 7);          // Far above target
    errorInput->addFuzzySet(veryNegative);
    errorInput->addFuzzySet(negative);
    errorInput->addFuzzySet(zero);
    errorInput->addFuzzySet(positive);
    errorInput->addFuzzySet(veryPositive);
    fuzzy->addFuzzyInput(errorInput);

    FuzzyInput *speedInput = new FuzzyInput(2);
    goingDown = new FuzzySet(-10, -10, -1.5, -0.3);   
    steady = new FuzzySet(-0.75, -0.1, 0.1, 0.75);    
    goingUp = new FuzzySet(0.3, 1.5, 10, 10);         
    speedInput->addFuzzySet(goingDown);
    speedInput->addFuzzySet(steady);
    speedInput->addFuzzySet(goingUp);
    fuzzy->addFuzzyInput(speedInput);

    FuzzyOutput *fanSpeed = new FuzzyOutput(1);
    veryLow = new FuzzySet(VERY_LOW_SPEED, VERY_LOW_SPEED, VERY_LOW_SPEED, VERY_LOW_SPEED);
    low = new FuzzySet(LOW_SPEED, LOW_SPEED, LOW_SPEED, LOW_SPEED);      
    medium = new FuzzySet(MID_SPEED, MID_SPEED, MID_SPEED, MID_SPEED);   
    high = new FuzzySet(HIGH_SPEED, HIGH_SPEED, HIGH_SPEED, HIGH_SPEED);     
    veryHigh = new FuzzySet(VERY_HIGH_SPEED, VERY_HIGH_SPEED, VERY_HIGH_SPEED, VERY_HIGH_SPEED); 
    fanSpeed->addFuzzySet(veryLow);
    fanSpeed->addFuzzySet(low);
    fanSpeed->addFuzzySet(medium);
    fanSpeed->addFuzzySet(high);
    fanSpeed->addFuzzySet(veryHigh);
    fuzzy->addFuzzyOutput(fanSpeed);

    // Rules based on error and speed
    // Very negative error (ball far below target)
    FuzzyRuleAntecedent *ifVeryNegativeAndGoingDown = new FuzzyRuleAntecedent();
    ifVeryNegativeAndGoingDown->joinWithAND(veryNegative, goingDown);
    FuzzyRuleConsequent *thenVeryHigh1 = new FuzzyRuleConsequent();
    thenVeryHigh1->addOutput(veryHigh);
    fuzzy->addFuzzyRule(new FuzzyRule(1, ifVeryNegativeAndGoingDown, thenVeryHigh1));

    FuzzyRuleAntecedent *ifVeryNegativeAndSteady = new FuzzyRuleAntecedent();
    ifVeryNegativeAndSteady->joinWithAND(veryNegative, steady);
    FuzzyRuleConsequent *thenVeryHigh2 = new FuzzyRuleConsequent();
    thenVeryHigh2->addOutput(veryHigh);
    fuzzy->addFuzzyRule(new FuzzyRule(2, ifVeryNegativeAndSteady, thenVeryHigh2));

    FuzzyRuleAntecedent *ifVeryNegativeAndGoingUp = new FuzzyRuleAntecedent();
    ifVeryNegativeAndGoingUp->joinWithAND(veryNegative, goingUp);
    FuzzyRuleConsequent *thenHigh1 = new FuzzyRuleConsequent();
    thenHigh1->addOutput(high);
    fuzzy->addFuzzyRule(new FuzzyRule(3, ifVeryNegativeAndGoingUp, thenHigh1));

    // Negative error (ball below target)
    FuzzyRuleAntecedent *ifNegativeAndGoingDown = new FuzzyRuleAntecedent();
    ifNegativeAndGoingDown->joinWithAND(negative, goingDown);
    FuzzyRuleConsequent *thenHigh2 = new FuzzyRuleConsequent();
    thenHigh2->addOutput(high);
    fuzzy->addFuzzyRule(new FuzzyRule(4, ifNegativeAndGoingDown, thenHigh2));

    FuzzyRuleAntecedent *ifNegativeAndSteady = new FuzzyRuleAntecedent();
    ifNegativeAndSteady->joinWithAND(negative, steady);
    FuzzyRuleConsequent *thenMedium1 = new FuzzyRuleConsequent();
    thenMedium1->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(5, ifNegativeAndSteady, thenMedium1));

    FuzzyRuleAntecedent *ifNegativeAndGoingUp = new FuzzyRuleAntecedent();
    ifNegativeAndGoingUp->joinWithAND(negative, goingUp);
    FuzzyRuleConsequent *thenLow1 = new FuzzyRuleConsequent();
    thenLow1->addOutput(low);
    fuzzy->addFuzzyRule(new FuzzyRule(6, ifNegativeAndGoingUp, thenLow1));

    // Zero error (ball at target)
    FuzzyRuleAntecedent *ifZeroAndGoingDown = new FuzzyRuleAntecedent();
    ifZeroAndGoingDown->joinWithAND(zero, goingDown);
    FuzzyRuleConsequent *thenMedium2 = new FuzzyRuleConsequent();
    thenMedium2->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(7, ifZeroAndGoingDown, thenMedium2));

    FuzzyRuleAntecedent *ifZeroAndSteady = new FuzzyRuleAntecedent();
    ifZeroAndSteady->joinWithAND(zero, steady);
    FuzzyRuleConsequent *thenMedium3 = new FuzzyRuleConsequent();
    thenMedium3->addOutput(medium); 
    fuzzy->addFuzzyRule(new FuzzyRule(8, ifZeroAndSteady, thenMedium3));

    FuzzyRuleAntecedent *ifZeroAndGoingUp = new FuzzyRuleAntecedent();
    ifZeroAndGoingUp->joinWithAND(zero, goingUp);
    FuzzyRuleConsequent *thenMedium4 = new FuzzyRuleConsequent();
    thenMedium4->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(9, ifZeroAndGoingUp, thenMedium4));

    FuzzyRuleAntecedent *ifPositiveAndGoingDown = new FuzzyRuleAntecedent();
    ifPositiveAndGoingDown->joinWithAND(positive, goingDown);
    FuzzyRuleConsequent *thenLow4 = new FuzzyRuleConsequent(); 
    thenLow4->addOutput(low);  
    fuzzy->addFuzzyRule(new FuzzyRule(10, ifPositiveAndGoingDown, thenLow4));

    FuzzyRuleAntecedent *ifPositiveAndSteady = new FuzzyRuleAntecedent();
    ifPositiveAndSteady->joinWithAND(positive, steady);
    FuzzyRuleConsequent *thenLow5 = new FuzzyRuleConsequent();
    thenLow5->addOutput(low);
    fuzzy->addFuzzyRule(new FuzzyRule(11, ifPositiveAndSteady, thenLow5));

    FuzzyRuleAntecedent *ifPositiveAndGoingUp = new FuzzyRuleAntecedent();
    ifPositiveAndGoingUp->joinWithAND(positive, goingUp);
    FuzzyRuleConsequent *thenVeryLow2 = new FuzzyRuleConsequent(); 
    thenVeryLow2->addOutput(veryLow); 
    fuzzy->addFuzzyRule(new FuzzyRule(12, ifPositiveAndGoingUp, thenVeryLow2));

    FuzzyRuleAntecedent *ifVeryPositiveAndGoingDown = new FuzzyRuleAntecedent();
    ifVeryPositiveAndGoingDown->joinWithAND(veryPositive, goingDown);
    FuzzyRuleConsequent *thenVeryLow3 = new FuzzyRuleConsequent(); 
    thenVeryLow3->addOutput(veryLow); 
    fuzzy->addFuzzyRule(new FuzzyRule(13, ifVeryPositiveAndGoingDown, thenVeryLow3));

    FuzzyRuleAntecedent *ifVeryPositiveAndSteady = new FuzzyRuleAntecedent();
    ifVeryPositiveAndSteady->joinWithAND(veryPositive, steady);
    FuzzyRuleConsequent *thenVeryLow4 = new FuzzyRuleConsequent();
    thenVeryLow4->addOutput(veryLow);
    fuzzy->addFuzzyRule(new FuzzyRule(14, ifVeryPositiveAndSteady, thenVeryLow4));

    FuzzyRuleAntecedent *ifVeryPositiveAndGoingUp = new FuzzyRuleAntecedent();
    ifVeryPositiveAndGoingUp->joinWithAND(veryPositive, goingUp);
    FuzzyRuleConsequent *thenVeryLow5 = new FuzzyRuleConsequent();
    thenVeryLow5->addOutput(veryLow);
    fuzzy->addFuzzyRule(new FuzzyRule(15, ifVeryPositiveAndGoingUp, thenVeryLow5));
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

            static float lastFanPWM = 79.5; // Start with medium value
            
            fuzzy->setInput(1, targetError);
            fuzzy->setInput(2, ballSpeed);
            fuzzy->fuzzify();
            float fanPWM = fuzzy->defuzzify(1);
            
            // Adaptive rate limiting - faster during large errors
            float maxChange = 0.2; // Normal rate limit
            if (abs(targetError) > 3.0) {
                maxChange = 0.2; // Faster response for large errors
            }
            
            // Apply rate limiting
            if (fanPWM > lastFanPWM + maxChange) {
                fanPWM = lastFanPWM + maxChange;
            } else if (fanPWM < lastFanPWM - maxChange) {
                fanPWM = lastFanPWM - maxChange;
            }
            
            lastFanPWM = fanPWM;
            
            #ifdef PRINTS
                Serial.print("\tFan speed: ");
                Serial.println(fanPWM);
            #endif

            // Apply fan power
            setFanSpeed(fanPWM);
        } else {
            Serial.println("Error: Measurement timeout");
        }
    } else {
        int joystickValue = analogRead(JOYSTICK_PIN);
        float pwmDutyCycle = mapfloat(joystickValue, 0.0, 1023.0, 50.0, 90.0);
        setFanSpeed(pwmDutyCycle);
        #ifdef PRINTS
            Serial.print("Manual Mode: \t Distance: ");
            Serial.print(smoothedDistance);
            Serial.print("\tFanspeed(%): ");
            Serial.println(pwmDutyCycle);
        #endif
    }
    timeb = millis();
    delay(5);
}
