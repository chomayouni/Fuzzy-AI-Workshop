#include <SimpleUltrasonic.h>
#include <Fuzzy.h>

#define PWM_D9_PIN 9  // Must be 9 or 10 (Timer1-based PWM)
#define JOYSTICK_PIN A0

// Ultrasonic Sensor (Trigger, Echo)
SimpleUltrasonic sensor(2, 3);

// Fuzzy Logic System
Fuzzy *fuzzy = new Fuzzy();
FuzzySet *veryNegativeError, *negativeError, *zeroError, *positiveError, *veryPositiveError;  // Error sets
FuzzySet *goingDownFast, *goingDown, *steady, *goingUp, *goingUpFast;  // Speed sets with 5 levels
FuzzySet *low, *medium, *high;   // Reduced to 3 output sets

// Toggle mode: 0 = Joystick (Manual), 1 = Fuzzy (Automatic)
bool fuzzyMode = true;

// Moving Average Filter for Smoothing Distance Measurements
float smoothedDistance = 0;
const int smoothingFactor = 10;  // Number of samples to average
float previousDistance = 0;
unsigned long previousTime, timea, timeb = 0;
float ballSpeed = 0;
float targetDistance = 9.0;  // Default target position

float mapfloat(float in, float in_min, float in_max, float out_min, float out_max) {
    return (((in - in_min) / (in_max - in_min)) * (out_max - out_min) + out_min);
}

void updateSmoothedDistance(float newReading) {
    // Avoid invalid readings
    if (newReading < 0) return;
    
    static float readings[smoothingFactor] = {0};
    static int index = 0;
    static bool bufferFilled = false;
    
    // Store new reading
    readings[index] = newReading;
    index = (index + 1) % smoothingFactor;
    if (index == 0) bufferFilled = true;
    
    // Calculate average
    float sum = 0;
    int count = bufferFilled ? smoothingFactor : index;
    for (int i = 0; i < count; i++) {
        sum += readings[i];
    }
    smoothedDistance = (count > 0) ? sum / count : newReading;
}

// Simplify speed calculation and invert the sign
void updateBallSpeed() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0;  // Convert to seconds

    // Only update if enough time has passed
    if (deltaTime >= 0.05) {
        // Calculate speed with simple filtering
        // Invert the sign: negative distance change (ball moving up) = positive speed
        float instantSpeed = (previousDistance - smoothedDistance) / deltaTime;
        
        // Simple low-pass filter with stronger filtering
        const float speedFilterAlpha = 0.2; // Lower value = more filtering
        ballSpeed = (speedFilterAlpha * instantSpeed) + ((1.0 - speedFilterAlpha) * ballSpeed);
        
        // Apply a dead zone for near-zero speeds
        if (abs(ballSpeed) < 0.5) {
            ballSpeed = 0.0;
        }
        
        // Constrain to realistic values
        ballSpeed = constrain(ballSpeed, -10.0, 10.0);
        
        previousDistance = smoothedDistance;
        previousTime = currentTime;
    }
}

void setupFuzzy() {
    // Define input fuzzy variable for error (target - actual)
    FuzzyInput *errorInput = new FuzzyInput(1);
    veryNegativeError = new FuzzySet(-14, -14, -10, -6);  // Ball far below target
    negativeError = new FuzzySet(-8, -5, -3, -1);         // Ball somewhat below target
    zeroError = new FuzzySet(-2, -0.5, 0.5, 2);           // Ball at target
    positiveError = new FuzzySet(1, 3, 5, 8);             // Ball somewhat above target
    veryPositiveError = new FuzzySet(6, 10, 14, 14);      // Ball far above target
    errorInput->addFuzzySet(veryNegativeError);
    errorInput->addFuzzySet(negativeError);
    errorInput->addFuzzySet(zeroError);
    errorInput->addFuzzySet(positiveError);
    errorInput->addFuzzySet(veryPositiveError);
    fuzzy->addFuzzyInput(errorInput);

    // Define input fuzzy variable for speed - expanded to 5 levels
    FuzzyInput *speedInput = new FuzzyInput(2);
    goingDownFast = new FuzzySet(3, 6, 10, 10);       // Falling quickly
    goingDown = new FuzzySet(.5, 2, 3, 4);             // Falling slowly
    steady = new FuzzySet(-0.75, 0, 0, 0.75);         // Near-zero speed
    goingUp = new FuzzySet(-4, -3, -2, -.5);           // Rising slowly
    goingUpFast = new FuzzySet(-10, -10, -6, -3);     // Rising quickly
    speedInput->addFuzzySet(goingDownFast);
    speedInput->addFuzzySet(goingDown);
    speedInput->addFuzzySet(steady);
    speedInput->addFuzzySet(goingUp);
    speedInput->addFuzzySet(goingUpFast);
    fuzzy->addFuzzyInput(speedInput);

    // Define output fuzzy variable - using 3 trapezoid sets for better defuzzification
    FuzzyOutput *fanSpeed = new FuzzyOutput(1);
    // Using trapezoid sets instead of singletons
    low = new FuzzySet(77, 79, 79, 81);         // Low fan speed - min to barely lift
    medium = new FuzzySet(80, 83, 83, 86);       // Medium fan speed - stable hover
    high = new FuzzySet(85, 87, 87, 89);         // High fan speed - strong lift
    fanSpeed->addFuzzySet(low);
    fanSpeed->addFuzzySet(medium);
    fanSpeed->addFuzzySet(high);
    fuzzy->addFuzzyOutput(fanSpeed);

    // ===== RULE SET (20 rules) =====
    // For very negative error (ball far below target) - needs high power
    FuzzyRuleAntecedent *ifVeryNegativeErrorAndGoingDownFast = new FuzzyRuleAntecedent();
    ifVeryNegativeErrorAndGoingDownFast->joinWithAND(veryNegativeError, goingDownFast);
    FuzzyRuleConsequent *thenHigh1 = new FuzzyRuleConsequent();
    thenHigh1->addOutput(high);
    fuzzy->addFuzzyRule(new FuzzyRule(1, ifVeryNegativeErrorAndGoingDownFast, thenHigh1));

    FuzzyRuleAntecedent *ifVeryNegativeErrorAndGoingDown = new FuzzyRuleAntecedent();
    ifVeryNegativeErrorAndGoingDown->joinWithAND(veryNegativeError, goingDown);
    FuzzyRuleConsequent *thenHigh2 = new FuzzyRuleConsequent();
    thenHigh2->addOutput(high);
    fuzzy->addFuzzyRule(new FuzzyRule(2, ifVeryNegativeErrorAndGoingDown, thenHigh2));

    FuzzyRuleAntecedent *ifVeryNegativeErrorAndSteady = new FuzzyRuleAntecedent();
    ifVeryNegativeErrorAndSteady->joinWithAND(veryNegativeError, steady);
    FuzzyRuleConsequent *thenHigh3 = new FuzzyRuleConsequent();
    thenHigh3->addOutput(high);
    fuzzy->addFuzzyRule(new FuzzyRule(3, ifVeryNegativeErrorAndSteady, thenHigh3));

    FuzzyRuleAntecedent *ifVeryNegativeErrorAndGoingUp = new FuzzyRuleAntecedent();
    ifVeryNegativeErrorAndGoingUp->joinWithAND(veryNegativeError, goingUp);
    FuzzyRuleConsequent *thenMedium1 = new FuzzyRuleConsequent();
    thenMedium1->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(4, ifVeryNegativeErrorAndGoingUp, thenMedium1));

    // For negative error (ball somewhat below target) - needs medium-high power
    FuzzyRuleAntecedent *ifNegativeErrorAndGoingDownFast = new FuzzyRuleAntecedent();
    ifNegativeErrorAndGoingDownFast->joinWithAND(negativeError, goingDownFast);
    FuzzyRuleConsequent *thenHigh4 = new FuzzyRuleConsequent();
    thenHigh4->addOutput(high);
    fuzzy->addFuzzyRule(new FuzzyRule(5, ifNegativeErrorAndGoingDownFast, thenHigh4));

    FuzzyRuleAntecedent *ifNegativeErrorAndGoingDown = new FuzzyRuleAntecedent();
    ifNegativeErrorAndGoingDown->joinWithAND(negativeError, goingDown);
    FuzzyRuleConsequent *thenMedium2 = new FuzzyRuleConsequent();
    thenMedium2->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(6, ifNegativeErrorAndGoingDown, thenMedium2));

    FuzzyRuleAntecedent *ifNegativeErrorAndSteady = new FuzzyRuleAntecedent();
    ifNegativeErrorAndSteady->joinWithAND(negativeError, steady);
    FuzzyRuleConsequent *thenMedium3 = new FuzzyRuleConsequent();
    thenMedium3->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(7, ifNegativeErrorAndSteady, thenMedium3));

    FuzzyRuleAntecedent *ifNegativeErrorAndGoingUp = new FuzzyRuleAntecedent();
    ifNegativeErrorAndGoingUp->joinWithAND(negativeError, goingUp);
    FuzzyRuleConsequent *thenMedium4 = new FuzzyRuleConsequent();
    thenMedium4->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(8, ifNegativeErrorAndGoingUp, thenMedium4));

    // For zero error (ball at target position) - needs medium-low power
    FuzzyRuleAntecedent *ifZeroErrorAndGoingDownFast = new FuzzyRuleAntecedent();
    ifZeroErrorAndGoingDownFast->joinWithAND(zeroError, goingDownFast);
    FuzzyRuleConsequent *thenMedium5 = new FuzzyRuleConsequent();
    thenMedium5->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(9, ifZeroErrorAndGoingDownFast, thenMedium5));

    FuzzyRuleAntecedent *ifZeroErrorAndGoingDown = new FuzzyRuleAntecedent();
    ifZeroErrorAndGoingDown->joinWithAND(zeroError, goingDown);
    FuzzyRuleConsequent *thenMedium6 = new FuzzyRuleConsequent();
    thenMedium6->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(10, ifZeroErrorAndGoingDown, thenMedium6));

    FuzzyRuleAntecedent *ifZeroErrorAndSteady = new FuzzyRuleAntecedent();
    ifZeroErrorAndSteady->joinWithAND(zeroError, steady);
    FuzzyRuleConsequent *thenMedium7 = new FuzzyRuleConsequent();
    thenMedium7->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(11, ifZeroErrorAndSteady, thenMedium7));

    FuzzyRuleAntecedent *ifZeroErrorAndGoingUp = new FuzzyRuleAntecedent();
    ifZeroErrorAndGoingUp->joinWithAND(zeroError, goingUp);
    FuzzyRuleConsequent *thenLow1 = new FuzzyRuleConsequent();
    thenLow1->addOutput(low);
    fuzzy->addFuzzyRule(new FuzzyRule(12, ifZeroErrorAndGoingUp, thenLow1));

    // For positive error (ball somewhat above target) - needs low power
    FuzzyRuleAntecedent *ifPositiveErrorAndGoingDownFast = new FuzzyRuleAntecedent();
    ifPositiveErrorAndGoingDownFast->joinWithAND(positiveError, goingDownFast);
    FuzzyRuleConsequent *thenLow2 = new FuzzyRuleConsequent();
    thenLow2->addOutput(low);
    fuzzy->addFuzzyRule(new FuzzyRule(13, ifPositiveErrorAndGoingDownFast, thenLow2));

    FuzzyRuleAntecedent *ifPositiveErrorAndGoingDown = new FuzzyRuleAntecedent();
    ifPositiveErrorAndGoingDown->joinWithAND(positiveError, goingDown);
    FuzzyRuleConsequent *thenLow3 = new FuzzyRuleConsequent();
    thenLow3->addOutput(low);
    fuzzy->addFuzzyRule(new FuzzyRule(14, ifPositiveErrorAndGoingDown, thenLow3));

    FuzzyRuleAntecedent *ifPositiveErrorAndSteady = new FuzzyRuleAntecedent();
    ifPositiveErrorAndSteady->joinWithAND(positiveError, steady);
    FuzzyRuleConsequent *thenLow4 = new FuzzyRuleConsequent();
    thenLow4->addOutput(low);
    fuzzy->addFuzzyRule(new FuzzyRule(15, ifPositiveErrorAndSteady, thenLow4));

    FuzzyRuleAntecedent *ifPositiveErrorAndGoingUp = new FuzzyRuleAntecedent();
    ifPositiveErrorAndGoingUp->joinWithAND(positiveError, goingUp);
    FuzzyRuleConsequent *thenMedium8 = new FuzzyRuleConsequent();
    thenMedium8->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(16, ifPositiveErrorAndGoingUp, thenMedium8));

    // For very positive error (ball far above target) - needs very low power
    FuzzyRuleAntecedent *ifVeryPositiveErrorAndGoingDownFast = new FuzzyRuleAntecedent();
    ifVeryPositiveErrorAndGoingDownFast->joinWithAND(veryPositiveError, goingDownFast);
    FuzzyRuleConsequent *thenLow5 = new FuzzyRuleConsequent();
    thenLow5->addOutput(low);
    fuzzy->addFuzzyRule(new FuzzyRule(17, ifVeryPositiveErrorAndGoingDownFast, thenLow5));

    FuzzyRuleAntecedent *ifVeryPositiveErrorAndGoingDown = new FuzzyRuleAntecedent();
    ifVeryPositiveErrorAndGoingDown->joinWithAND(veryPositiveError, goingDown);
    FuzzyRuleConsequent *thenLow6 = new FuzzyRuleConsequent();
    thenLow6->addOutput(low);
    fuzzy->addFuzzyRule(new FuzzyRule(18, ifVeryPositiveErrorAndGoingDown, thenLow6));

    FuzzyRuleAntecedent *ifVeryPositiveErrorAndSteady = new FuzzyRuleAntecedent();
    ifVeryPositiveErrorAndSteady->joinWithAND(veryPositiveError, steady);
    FuzzyRuleConsequent *thenLow7 = new FuzzyRuleConsequent();
    thenLow7->addOutput(low);
    fuzzy->addFuzzyRule(new FuzzyRule(19, ifVeryPositiveErrorAndSteady, thenLow7));

    FuzzyRuleAntecedent *ifVeryPositiveErrorAndGoingUp = new FuzzyRuleAntecedent();
    ifVeryPositiveErrorAndGoingUp->joinWithAND(veryPositiveError, goingUp);
    FuzzyRuleConsequent *thenMedium9 = new FuzzyRuleConsequent();
    thenMedium9->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(20, ifVeryPositiveErrorAndGoingUp, thenMedium9));
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
}

void loop() {
    timea = millis();
    
    // Fixed target position instead of using joystick
    targetDistance = 9.0;  // Middle of the tube
    
    float distance = sensor.measureDistanceInInch();
    updateSmoothedDistance(distance);
    updateBallSpeed();  // Update ball speed with improved filtering

    if (distance != -1) {
        // Calculate error (target - actual)
        float error = targetDistance - smoothedDistance;
        
        Serial.print("Target: ");
        Serial.print(targetDistance);
        Serial.print(", Actual: ");
        Serial.print(smoothedDistance);
        Serial.print(", Error: ");
        Serial.print(error);
        Serial.print(", Speed: ");
        Serial.print(ballSpeed);
        Serial.println(" in/sec");

        // Apply fuzzy control
        fuzzy->setInput(1, error);
        fuzzy->setInput(2, ballSpeed);
        fuzzy->fuzzify();
        
//        // Debug output for input and output fuzzy sets
//        Serial.println("Error membership values:");
//        Serial.print("veryNegativeError: ");
//        Serial.println(veryNegativeError->getPertinence());
//        Serial.print("negativeError: ");
//        Serial.println(negativeError->getPertinence());
//        Serial.print("zeroError: ");
//        Serial.println(zeroError->getPertinence());
//        Serial.print("positiveError: ");
//        Serial.println(positiveError->getPertinence());
//        Serial.print("veryPositiveError: ");
//        Serial.println(veryPositiveError->getPertinence());
//        
//        Serial.println("Speed membership values:");
//        Serial.print("goingDownFast: ");
//        Serial.println(goingDownFast->getPertinence());
//        Serial.print("goingDown: ");
//        Serial.println(goingDown->getPertinence());
//        Serial.print("steady: ");
//        Serial.println(steady->getPertinence());
//        Serial.print("goingUp: ");
//        Serial.println(goingUp->getPertinence());
//        Serial.print("goingUpFast: ");
//        Serial.println(goingUpFast->getPertinence());
        
//        // Debug output for rule activations
//        Serial.println("Rule activations:");
//        bool anyRuleActive = false;
//        for (int i = 1; i <= 20; i++) {  // Now using 20 rules
//            float activation = fuzzy->isFiredRule(i);
//            if (activation > 0) {
//                anyRuleActive = true;
//                Serial.print("Rule ");
//                Serial.print(i);
//                Serial.print(": ");
//                Serial.println(activation);
//            }
//        }
//        
//        if (!anyRuleActive) {
//            Serial.println("WARNING: No rules are active!");
//        }
        
        // Get defuzzification result
        float fanPWM = fuzzy->defuzzify(1);
        
        // Apply rate limiting to prevent overshooting
        static float lastFanSpeed = 85.0;  // Start at medium
        float maxChange = 0.5;  // Maximum change per iteration
        
        if (fanPWM > lastFanSpeed + maxChange) {
            fanPWM = lastFanSpeed + maxChange;
        } else if (fanPWM < lastFanSpeed - maxChange) {
            fanPWM = lastFanSpeed - maxChange;
        }
        
        lastFanSpeed = fanPWM;
        
        Serial.print("Fan speed: ");
        Serial.println(fanPWM);
        setFanSpeed(fanPWM);
    } else {
        Serial.println("Error: Measurement timeout");
    }
    
    timeb = millis();
    delay(10);
}
