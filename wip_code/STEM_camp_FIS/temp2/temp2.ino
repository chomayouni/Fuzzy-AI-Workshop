#include <SimpleUltrasonic.h>
#include <Fuzzy.h>

#define PWM_D9_PIN 9  // Must be 9 or 10 (Timer1-based PWM)
#define JOYSTICK_PIN A0

// Ultrasonic Sensor (Trigger, Echo)
SimpleUltrasonic sensor(2, 3);

// Fuzzy Logic System
Fuzzy *fuzzy = new Fuzzy();
FuzzySet *veryNegativeError, *negativeError, *zeroError, *positiveError, *veryPositiveError;  // Error sets
FuzzySet *goingDown, *steady, *goingUp;  // Reduced speed sets (3 instead of 5)
FuzzySet *veryLow, *low, *medium, *high, *veryHigh;   // 5 output sets for better granularity

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

    // Define input fuzzy variable for speed 
    FuzzyInput *speedInput = new FuzzyInput(2);
    goingDown = new FuzzySet(1, 3, 10, 10);             // Falling (combined slow and fast)
    steady = new FuzzySet(-1, 0, 0, 1);                 // Near-zero speed (slightly widened)
    goingUp = new FuzzySet(-10, -10, -3, -1);           // Rising (combined slow and fast)
    speedInput->addFuzzySet(goingDown);
    speedInput->addFuzzySet(steady);
    speedInput->addFuzzySet(goingUp);
    fuzzy->addFuzzyInput(speedInput);

    // Define output fuzzy variable - using 5 trapezoid sets for better control
    FuzzyOutput *fanSpeed = new FuzzyOutput(1);
    veryLow = new FuzzySet(75, 78, 78, 80);         // Very low power - let ball fall
    low = new FuzzySet(79, 81, 81, 83);             // Low power - gentle descent
    medium = new FuzzySet(82, 84, 84, 86);          // Medium power - hovering
    high = new FuzzySet(85, 87, 87, 89);            // High power - gentle ascent
    veryHigh = new FuzzySet(88, 90, 90, 92);        // Very high power - rapid ascent
    fanSpeed->addFuzzySet(veryLow);
    fanSpeed->addFuzzySet(low);
    fanSpeed->addFuzzySet(medium);
    fanSpeed->addFuzzySet(high);
    fanSpeed->addFuzzySet(veryHigh);
    fuzzy->addFuzzyOutput(fanSpeed);

    // RULES (5 error states × 3 speed states = 15 rules, well under limit)
    
    // Very negative error (ball far below target)
    FuzzyRuleAntecedent *ifVeryNegativeAndGoingDown = new FuzzyRuleAntecedent();
    ifVeryNegativeAndGoingDown->joinWithAND(veryNegativeError, goingDown);
    FuzzyRuleConsequent *thenVeryHigh1 = new FuzzyRuleConsequent();
    thenVeryHigh1->addOutput(veryHigh);
    fuzzy->addFuzzyRule(new FuzzyRule(1, ifVeryNegativeAndGoingDown, thenVeryHigh1));
    
    FuzzyRuleAntecedent *ifVeryNegativeAndSteady = new FuzzyRuleAntecedent();
    ifVeryNegativeAndSteady->joinWithAND(veryNegativeError, steady);
    FuzzyRuleConsequent *thenVeryHigh2 = new FuzzyRuleConsequent();
    thenVeryHigh2->addOutput(veryHigh);
    fuzzy->addFuzzyRule(new FuzzyRule(2, ifVeryNegativeAndSteady, thenVeryHigh2));
    
    FuzzyRuleAntecedent *ifVeryNegativeAndGoingUp = new FuzzyRuleAntecedent();
    ifVeryNegativeAndGoingUp->joinWithAND(veryNegativeError, goingUp);
    FuzzyRuleConsequent *thenHigh1 = new FuzzyRuleConsequent();
    thenHigh1->addOutput(high);
    fuzzy->addFuzzyRule(new FuzzyRule(3, ifVeryNegativeAndGoingUp, thenHigh1));
    
    // Negative error (ball somewhat below target)
    FuzzyRuleAntecedent *ifNegativeAndGoingDown = new FuzzyRuleAntecedent();
    ifNegativeAndGoingDown->joinWithAND(negativeError, goingDown);
    FuzzyRuleConsequent *thenHigh2 = new FuzzyRuleConsequent();
    thenHigh2->addOutput(high);
    fuzzy->addFuzzyRule(new FuzzyRule(4, ifNegativeAndGoingDown, thenHigh2));
    
    FuzzyRuleAntecedent *ifNegativeAndSteady = new FuzzyRuleAntecedent();
    ifNegativeAndSteady->joinWithAND(negativeError, steady);
    FuzzyRuleConsequent *thenMedium1 = new FuzzyRuleConsequent();
    thenMedium1->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(5, ifNegativeAndSteady, thenMedium1));
    
    FuzzyRuleAntecedent *ifNegativeAndGoingUp = new FuzzyRuleAntecedent();
    ifNegativeAndGoingUp->joinWithAND(negativeError, goingUp);
    FuzzyRuleConsequent *thenMedium2 = new FuzzyRuleConsequent();
    thenMedium2->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(6, ifNegativeAndGoingUp, thenMedium2));
    
    // Zero error (ball at target)
    FuzzyRuleAntecedent *ifZeroAndGoingDown = new FuzzyRuleAntecedent();
    ifZeroAndGoingDown->joinWithAND(zeroError, goingDown);
    FuzzyRuleConsequent *thenMedium3 = new FuzzyRuleConsequent();
    thenMedium3->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(7, ifZeroAndGoingDown, thenMedium3));
    
    FuzzyRuleAntecedent *ifZeroAndSteady = new FuzzyRuleAntecedent();
    ifZeroAndSteady->joinWithAND(zeroError, steady);
    FuzzyRuleConsequent *thenMedium4 = new FuzzyRuleConsequent();
    thenMedium4->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(8, ifZeroAndSteady, thenMedium4));
    
    FuzzyRuleAntecedent *ifZeroAndGoingUp = new FuzzyRuleAntecedent();
    ifZeroAndGoingUp->joinWithAND(zeroError, goingUp);
    FuzzyRuleConsequent *thenLow1 = new FuzzyRuleConsequent();
    thenLow1->addOutput(low);
    fuzzy->addFuzzyRule(new FuzzyRule(9, ifZeroAndGoingUp, thenLow1));
    
    // Positive error (ball somewhat above target)
    FuzzyRuleAntecedent *ifPositiveAndGoingDown = new FuzzyRuleAntecedent();
    ifPositiveAndGoingDown->joinWithAND(positiveError, goingDown);
    FuzzyRuleConsequent *thenVeryLow1 = new FuzzyRuleConsequent();
    thenVeryLow1->addOutput(veryLow);
    fuzzy->addFuzzyRule(new FuzzyRule(10, ifPositiveAndGoingDown, thenVeryLow1));
    
    FuzzyRuleAntecedent *ifPositiveAndSteady = new FuzzyRuleAntecedent();
    ifPositiveAndSteady->joinWithAND(positiveError, steady);
    FuzzyRuleConsequent *thenLow2 = new FuzzyRuleConsequent();
    thenLow2->addOutput(low);
    fuzzy->addFuzzyRule(new FuzzyRule(11, ifPositiveAndSteady, thenLow2));
    
    FuzzyRuleAntecedent *ifPositiveAndGoingUp = new FuzzyRuleAntecedent();
    ifPositiveAndGoingUp->joinWithAND(positiveError, goingUp);
    FuzzyRuleConsequent *thenMedium5 = new FuzzyRuleConsequent();
    thenMedium5->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(12, ifPositiveAndGoingUp, thenMedium5));
    
    // Very positive error (ball far above target)
    FuzzyRuleAntecedent *ifVeryPositiveAndGoingDown = new FuzzyRuleAntecedent();
    ifVeryPositiveAndGoingDown->joinWithAND(veryPositiveError, goingDown);
    FuzzyRuleConsequent *thenVeryLow2 = new FuzzyRuleConsequent();
    thenVeryLow2->addOutput(veryLow);
    fuzzy->addFuzzyRule(new FuzzyRule(13, ifVeryPositiveAndGoingDown, thenVeryLow2));
    
    FuzzyRuleAntecedent *ifVeryPositiveAndSteady = new FuzzyRuleAntecedent();
    ifVeryPositiveAndSteady->joinWithAND(veryPositiveError, steady);
    FuzzyRuleConsequent *thenVeryLow3 = new FuzzyRuleConsequent();
    thenVeryLow3->addOutput(veryLow);
    fuzzy->addFuzzyRule(new FuzzyRule(14, ifVeryPositiveAndSteady, thenVeryLow3));
    
    FuzzyRuleAntecedent *ifVeryPositiveAndGoingUp = new FuzzyRuleAntecedent();
    ifVeryPositiveAndGoingUp->joinWithAND(veryPositiveError, goingUp);
    FuzzyRuleConsequent *thenLow3 = new FuzzyRuleConsequent();
    thenLow3->addOutput(low);
    fuzzy->addFuzzyRule(new FuzzyRule(15, ifVeryPositiveAndGoingUp, thenLow3));
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
    targetDistance = 8.0;  // Middle of the tube
    
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
        
        // Get defuzzification result
        float fanPWM = fuzzy->defuzzify(1);
        
        // Apply rate limiting to prevent overshooting
        static float lastFanSpeed = 85.0;  // Start at medium
        float maxChange = 10.0;  // Maximum change per iteration
        
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
