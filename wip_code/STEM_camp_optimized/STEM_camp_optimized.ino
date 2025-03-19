#include <SimpleUltrasonic.h>
#include <Fuzzy.h>

#define PWM_D9_PIN 9  // Must be 9 or 10 (Timer1-based PWM)
#define JOYSTICK_PIN A0

// Ultrasonic Sensor (Trigger, Echo)
SimpleUltrasonic sensor(2, 3);

// Fuzzy Logic System
Fuzzy fuzzy;

// Toggle mode: 0 = Joystick (Manual), 1 = Fuzzy (Automatic)
bool fuzzyMode = true;

int smoothedDistance = 0;
const int smoothingFactor = 5;
int readings[smoothingFactor] = {0};
int index = 0;

void updateSmoothedDistance(int newReading) {
    readings[index] = newReading;
    index = (index + 1) % smoothingFactor;

    int sum = 0;
    for (int i = 0; i < smoothingFactor; i++) {
        sum += readings[i];
    }
    smoothedDistance = sum / smoothingFactor;
}

void setupFuzzy() {
    static FuzzyInput distanceInput(1);
    static FuzzySet veryClose(2, 3, 3, 5), close(4, 6, 6, 8), mid(7, 10, 10, 12), far(11, 14, 14, 16), veryFar(15, 16, 16, 16);
    distanceInput.addFuzzySet(&veryClose);
    distanceInput.addFuzzySet(&close);
    distanceInput.addFuzzySet(&mid);
    distanceInput.addFuzzySet(&far);
    distanceInput.addFuzzySet(&veryFar);
    fuzzy.addFuzzyInput(&distanceInput);

    static FuzzyOutput fanSpeed(1);
    static FuzzySet low(70, 72, 73, 75), mediumLow(74, 76, 77, 79), medium(78, 80, 81, 83), mediumHigh(82, 83, 84, 85), high(85, 85, 85, 85);
    fanSpeed.addFuzzySet(&low);
    fanSpeed.addFuzzySet(&mediumLow);
    fanSpeed.addFuzzySet(&medium);
    fanSpeed.addFuzzySet(&mediumHigh);
    fanSpeed.addFuzzySet(&high);
    fuzzy.addFuzzyOutput(&fanSpeed);

    static FuzzyRuleAntecedent ifVeryClose;
    ifVeryClose.joinSingle(&veryClose);
    static FuzzyRuleConsequent thenLow;
    thenLow.addOutput(&low);
    fuzzy.addFuzzyRule(new FuzzyRule(1, &ifVeryClose, &thenLow));

    static FuzzyRuleAntecedent ifClose;
    ifClose.joinSingle(&close);
    static FuzzyRuleConsequent thenMediumLow;
    thenMediumLow.addOutput(&mediumLow);
    fuzzy.addFuzzyRule(new FuzzyRule(2, &ifClose, &thenMediumLow));

    static FuzzyRuleAntecedent ifMid;
    ifMid.joinSingle(&mid);
    static FuzzyRuleConsequent thenMedium;
    thenMedium.addOutput(&medium);
    fuzzy.addFuzzyRule(new FuzzyRule(3, &ifMid, &thenMedium));

    static FuzzyRuleAntecedent ifFar;
    ifFar.joinSingle(&far);
    static FuzzyRuleConsequent thenMediumHigh;
    thenMediumHigh.addOutput(&mediumHigh);
    fuzzy.addFuzzyRule(new FuzzyRule(4, &ifFar, &thenMediumHigh));

    static FuzzyRuleAntecedent ifVeryFar;
    ifVeryFar.joinSingle(&veryFar);
    static FuzzyRuleConsequent thenHigh;
    thenHigh.addOutput(&high);
    fuzzy.addFuzzyRule(new FuzzyRule(5, &ifVeryFar, &thenHigh));
}

void setupPWM() {
    pinMode(PWM_D9_PIN, OUTPUT);
    TCCR1A = _BV(COM1A1) | _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
    ICR1 = 320;
}

void setFanSpeed(int percentage) {
    OCR1A = (percentage * 320) / 100;
}

void setup() {
    Serial.begin(115200);
    sensor.begin();
    setupFuzzy();
    setupPWM();
}

void loop() {
    static int lastFanPWM = -1;
    static int lastJoystickValue = -1;

    if (fuzzyMode) {
        int distance = sensor.measureDistanceInInch();
        if (distance != -1) {
            updateSmoothedDistance(distance);
            fuzzy.setInput(1, smoothedDistance);
            fuzzy.fuzzify();
            int fanPWM = (int)fuzzy.defuzzify(1);
            
            if (fanPWM != lastFanPWM) {
                setFanSpeed(fanPWM);
                lastFanPWM = fanPWM;
                Serial.print("Distance: ");
                Serial.print(smoothedDistance);
                Serial.print(" inches, Fan PWM (Fuzzy): ");
                Serial.println(fanPWM);
            }
        }
    } else {
        int joystickValue = analogRead(JOYSTICK_PIN);
        int pwmDutyCycle = (joystickValue * 100) / 1023;

        if (joystickValue != lastJoystickValue) {
            setFanSpeed(pwmDutyCycle);
            lastJoystickValue = joystickValue;
            Serial.print("Joystick: ");
            Serial.print(joystickValue);
            Serial.print(" -> Fan PWM: ");
            Serial.println(pwmDutyCycle);
        }
    }
}
