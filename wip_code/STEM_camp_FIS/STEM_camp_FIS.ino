#include <SimpleUltrasonic.h>
#include <Fuzzy.h>

#define PWM_D9_PIN 9  // Must be 9 or 10 (Timer1-based PWM)
#define JOYSTICK_PIN A0

// Ultrasonic Sensor (Trigger, Echo)
SimpleUltrasonic sensor(2, 3);

// Fuzzy Logic System
Fuzzy *fuzzy = new Fuzzy();
FuzzySet *veryClose, *close, *mid, *far, *veryFar;
FuzzySet *low, *mediumLow, *medium, *mediumHigh, *high;

// Toggle mode: 0 = Joystick (Manual), 1 = Fuzzy (Automatic)
bool fuzzyMode = false;

float mapfloat(float in, float in_min, float in_max, float out_min, float out_max) {
    return (((in - in_min) / (in_max - in_min)) * (out_max - out_min) + out_min);
}

// Moving Average Filter for Smoothing Distance Measurements
float smoothedDistance = 0;
const int smoothingFactor = 5; // Number of samples to average

void updateSmoothedDistance(float newReading) {
    static float readings[smoothingFactor] = {0};
    static int index = 0;

    readings[index] = newReading;
    index = (index + 1) % smoothingFactor;

    float sum = 0;
    for (int i = 0; i < smoothingFactor; i++) {
        sum += readings[i];
    }
    smoothedDistance = sum / smoothingFactor;
}

void setupFuzzy() {
    // Define input fuzzy variable
    FuzzyInput *distanceInput = new FuzzyInput(1);
    veryClose = new FuzzySet(2, 2, 4, 7);    // Extended overlap with "close"
    close = new FuzzySet(5, 6, 8, 10);       // Smooth blend with "mid"
    mid = new FuzzySet(9, 9.5, 10.5, 11);    // Very tight, precise goal region
    far = new FuzzySet(10, 12, 13, 15);      // Good overlap with "mid" & "veryFar"
    veryFar = new FuzzySet(14, 16, 16, 16);  // Extended left to blend well



    distanceInput->addFuzzySet(veryClose);
    distanceInput->addFuzzySet(close);
    distanceInput->addFuzzySet(mid);
    distanceInput->addFuzzySet(far);
    distanceInput->addFuzzySet(veryFar);
    fuzzy->addFuzzyInput(distanceInput);

    // Define output fuzzy variable
    FuzzyOutput *fanSpeed = new FuzzyOutput(1);
//    static FuzzySet low(70, 72, 73, 75), mediumLow(74, 76, 77, 79), medium(78, 79, 80, 81), mediumHigh(80, 82, 83, 84), high(85, 85, 85, 85);


  
    low = new FuzzySet(72, 72, 73, 75);
    mediumLow = new FuzzySet(74, 76, 77, 79);
    medium = new FuzzySet(78, 80, 81, 83);
    mediumHigh = new FuzzySet(82, 83, 83, 83);
    high = new FuzzySet(83, 84, 84, 84);
    fanSpeed->addFuzzySet(low);
    fanSpeed->addFuzzySet(mediumLow);
    fanSpeed->addFuzzySet(medium);
    fanSpeed->addFuzzySet(mediumHigh);
    fanSpeed->addFuzzySet(high);
    fuzzy->addFuzzyOutput(fanSpeed);

    // Define fuzzy rules
    FuzzyRuleAntecedent *ifVeryClose = new FuzzyRuleAntecedent();
    ifVeryClose->joinSingle(veryClose);
    FuzzyRuleConsequent *thenLow = new FuzzyRuleConsequent();
    thenLow->addOutput(low);
    fuzzy->addFuzzyRule(new FuzzyRule(1, ifVeryClose, thenLow));

    FuzzyRuleAntecedent *ifClose = new FuzzyRuleAntecedent();
    ifClose->joinSingle(close);
    FuzzyRuleConsequent *thenMediumLow = new FuzzyRuleConsequent();
    thenMediumLow->addOutput(mediumLow);
    fuzzy->addFuzzyRule(new FuzzyRule(2, ifClose, thenMediumLow));

    FuzzyRuleAntecedent *ifMid = new FuzzyRuleAntecedent();
    ifMid->joinSingle(mid);
    FuzzyRuleConsequent *thenMedium = new FuzzyRuleConsequent();
    thenMedium->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(3, ifMid, thenMedium));

    FuzzyRuleAntecedent *ifFar = new FuzzyRuleAntecedent();
    ifFar->joinSingle(far);
    FuzzyRuleConsequent *thenMediumHigh = new FuzzyRuleConsequent();
    thenMediumHigh->addOutput(mediumHigh);
    fuzzy->addFuzzyRule(new FuzzyRule(4, ifFar, thenMediumHigh));

    FuzzyRuleAntecedent *ifVeryFar = new FuzzyRuleAntecedent();
    ifVeryFar->joinSingle(veryFar);
    FuzzyRuleConsequent *thenHigh = new FuzzyRuleConsequent();
    thenHigh->addOutput(high);
    fuzzy->addFuzzyRule(new FuzzyRule(5, ifVeryFar, thenHigh));
}

void setupPWM() {
    pinMode(PWM_D9_PIN, OUTPUT);
    
    // Configure Timer1 for 25kHz PWM on Pin 9
    TCCR1A = _BV(COM1A1) | _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
    ICR1 = 320;  // 25kHz frequency
}

void setFanSpeed(float percentage) {
    OCR1A = (percentage / 100.0) * 320; // Convert 0-100% to Timer1 PWM value
}

void setup() {
    Serial.begin(115200);
    sensor.begin();
    setupFuzzy();
    setupPWM();
}

void loop() {
    float distance = sensor.measureDistanceInInch();
    updateSmoothedDistance(distance);
    if (fuzzyMode) {
        if (distance != -1) {
            Serial.print("Distance: ");
            Serial.print(smoothedDistance);
            Serial.println(" inches");

            fuzzy->setInput(1, smoothedDistance);
            fuzzy->fuzzify();
            float fanPWM = fuzzy->defuzzify(1);

            setFanSpeed(fanPWM);

//            Serial.print("Fan PWM (Fuzzy): ");
//            Serial.println(fanPWM);
        } else {
            Serial.println("Error: Measurement timeout");
        }
    } 
    
    
    else {
        int joystickValue = analogRead(JOYSTICK_PIN);
        float pwmDutyCycle = mapfloat(joystickValue, 0.0, 1023.0, 50.0, 90.0);

        setFanSpeed(pwmDutyCycle);

//        Serial.print("Joystick: ");
//        Serial.print(joystickValue);
//        Serial.print(" -> Fan PWM: ");
//        Serial.println(pwmDutyCycle);
        Serial.print("Distance: ");
        Serial.println(smoothedDistance);
        
    }

//    delay(100);
}
