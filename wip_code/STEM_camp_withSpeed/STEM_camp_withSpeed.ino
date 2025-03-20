#include <SimpleUltrasonic.h>
#include <Fuzzy.h>

#define PWM_D9_PIN 9  // Must be 9 or 10 (Timer1-based PWM)
#define JOYSTICK_PIN A0

// Ultrasonic Sensor (Trigger, Echo)
SimpleUltrasonic sensor(2, 3);

// Fuzzy Logic System
Fuzzy *fuzzy = new Fuzzy();
FuzzySet *veryClose, *close, *mid, *far, *veryFar;
FuzzySet *veryLow, *low, *mediumLow, *medium, *mediumHigh, *high, *veryHigh;
FuzzySet *goingDownFast, *goingDown, *steady, *goingUp, *goingUpFast;

// Toggle mode: 0 = Joystick (Manual), 1 = Fuzzy (Automatic)
bool fuzzyMode = true;

// Moving Average Filter for Smoothing Distance Measurements
float smoothedDistance = 0;
const int smoothingFactor = 10;  // Number of samples to average
float previousDistance = 0;
unsigned long previousTime, timea, timeb = 0;
float ballSpeed = 0;

float mapfloat(float in, float in_min, float in_max, float out_min, float out_max) {
    return (((in - in_min) / (in_max - in_min)) * (out_max - out_min) + out_min);
}

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

// Estimate ball speed (change in distance over time)
void updateBallSpeed() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0;  // Convert to seconds

    if (deltaTime > 0) {
        ballSpeed = (smoothedDistance - previousDistance) / deltaTime;
        ballSpeed = constrain(ballSpeed, -10.0, 10.0);
    }

    previousDistance = smoothedDistance;
    previousTime = currentTime;
    
}
void setupFuzzy() {
    // Define input fuzzy variable for distance (range: 2-16)
    FuzzyInput *distanceInput = new FuzzyInput(1);
    far = new FuzzySet(8.5, 11, 11, 16);    
    mid = new FuzzySet(8, 9, 9, 10);          
    close = new FuzzySet(2, 7, 7, 9.5);       
    distanceInput->addFuzzySet(far);
    distanceInput->addFuzzySet(mid);
    distanceInput->addFuzzySet(close);
    fuzzy->addFuzzyInput(distanceInput);

    // Define input fuzzy variable for speed - simpler speed detection
    FuzzyInput *speedInput = new FuzzyInput(2);
    goingDown = new FuzzySet(.5, 1.5, 10, 10);   // Falling
    steady = new FuzzySet(-.75, 0, 0, .75);       // Near-zero speed
    goingUp = new FuzzySet(-10, -10, -1.5, -.5); // Rising
    speedInput->addFuzzySet(goingDown);
    speedInput->addFuzzySet(steady);
    speedInput->addFuzzySet(goingUp);
    fuzzy->addFuzzyInput(speedInput);

    // Define output fuzzy variable (fan speed) - Range: 72-84
    FuzzyOutput *fanSpeed = new FuzzyOutput(1);
//    low = new FuzzySet(72, 76, 76, 80);      // Low fan speed
//    medium = new FuzzySet(78, 80, 80, 82);   // Medium fan speed
//    high = new FuzzySet(80, 83, 83, 83);     // High fan speed
    veryLow = new FuzzySet(75, 75, 75, 75);
    low = new FuzzySet(79, 79, 79, 79);
    medium = new FuzzySet(81, 81, 81, 81);
    high = new FuzzySet(84, 84, 84, 84);
    veryHigh = new FuzzySet(87, 87, 87, 87);
    fanSpeed->addFuzzySet(veryLow);
    fanSpeed->addFuzzySet(low);
    fanSpeed->addFuzzySet(medium);
    fanSpeed->addFuzzySet(high);
    fanSpeed->addFuzzySet(veryHigh);
    fuzzy->addFuzzyOutput(fanSpeed);

    // Simple rules that focus on stability
    
    // Rules for far distance
    FuzzyRuleAntecedent *ifFarAndGoingDown = new FuzzyRuleAntecedent();
    ifFarAndGoingDown->joinWithAND(far, goingDown);
    FuzzyRuleConsequent *thenHigh1 = new FuzzyRuleConsequent();
    thenHigh1->addOutput(veryHigh);
    fuzzy->addFuzzyRule(new FuzzyRule(1, ifFarAndGoingDown, thenHigh1));

    FuzzyRuleAntecedent *ifFarAndSteady = new FuzzyRuleAntecedent();
    ifFarAndSteady->joinWithAND(far, steady);
    FuzzyRuleConsequent *thenHigh2 = new FuzzyRuleConsequent();
    thenHigh2->addOutput(veryHigh);
    fuzzy->addFuzzyRule(new FuzzyRule(2, ifFarAndSteady, thenHigh2));

    FuzzyRuleAntecedent *ifFarAndGoingUp = new FuzzyRuleAntecedent();
    ifFarAndGoingUp->joinWithAND(far, goingUp);
    FuzzyRuleConsequent *thenMedium1 = new FuzzyRuleConsequent();
    thenMedium1->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(3, ifFarAndGoingUp, thenMedium1));

    // Rules for mid distance - focus on stability
    FuzzyRuleAntecedent *ifMidAndGoingDown = new FuzzyRuleAntecedent();
    ifMidAndGoingDown->joinWithAND(mid, goingDown);
    FuzzyRuleConsequent *thenMedium2 = new FuzzyRuleConsequent();
    thenMedium2->addOutput(high);
    fuzzy->addFuzzyRule(new FuzzyRule(4, ifMidAndGoingDown, thenMedium2));

    FuzzyRuleAntecedent *ifMidAndSteady = new FuzzyRuleAntecedent();
    ifMidAndSteady->joinWithAND(mid, steady);
    FuzzyRuleConsequent *thenMedium3 = new FuzzyRuleConsequent();
    thenMedium3->addOutput(medium); 
    fuzzy->addFuzzyRule(new FuzzyRule(5, ifMidAndSteady, thenMedium3));

    FuzzyRuleAntecedent *ifMidAndGoingUp = new FuzzyRuleAntecedent();
    ifMidAndGoingUp->joinWithAND(mid, goingUp);
    FuzzyRuleConsequent *thenMedium4 = new FuzzyRuleConsequent();
    thenMedium4->addOutput(low);
    fuzzy->addFuzzyRule(new FuzzyRule(6, ifMidAndGoingUp, thenMedium4));

    // Rules for close distance
    FuzzyRuleAntecedent *ifCloseAndGoingDown = new FuzzyRuleAntecedent();
    ifCloseAndGoingDown->joinWithAND(close, goingDown);
    FuzzyRuleConsequent *thenMedium5 = new FuzzyRuleConsequent();
    thenMedium5->addOutput(high);
    fuzzy->addFuzzyRule(new FuzzyRule(7, ifCloseAndGoingDown, thenMedium5));

    FuzzyRuleAntecedent *ifCloseAndSteady = new FuzzyRuleAntecedent();
    ifCloseAndSteady->joinWithAND(close, steady);
    FuzzyRuleConsequent *thenLow1 = new FuzzyRuleConsequent();
    thenLow1->addOutput(medium);
    fuzzy->addFuzzyRule(new FuzzyRule(8, ifCloseAndSteady, thenLow1));

    FuzzyRuleAntecedent *ifCloseAndGoingUp = new FuzzyRuleAntecedent();
    ifCloseAndGoingUp->joinWithAND(close, goingUp);
    FuzzyRuleConsequent *thenLow2 = new FuzzyRuleConsequent();
    thenLow2->addOutput(low);
    fuzzy->addFuzzyRule(new FuzzyRule(9, ifCloseAndGoingUp, thenLow2));
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
    if (fuzzyMode) {
        float distance = sensor.measureDistanceInInch();
        updateSmoothedDistance(distance);
        updateBallSpeed();  // Update ball speed based on change in distance

        if (distance != -1) {
            Serial.print("Distance: ");
            Serial.print(smoothedDistance);
            Serial.print(" inches, Speed: ");
            Serial.print(ballSpeed);
            Serial.println(" in/sec");
            

            fuzzy->setInput(1, smoothedDistance);
            fuzzy->setInput(2, ballSpeed);
            fuzzy->fuzzify();
            float fanPWM = fuzzy->defuzzify(1);
            Serial.print("Fan speed: ");
            Serial.println(fanPWM);
            setFanSpeed(fanPWM);
        } else {
            Serial.println("Error: Measurement timeout");
        }
    } else {
        int joystickValue = analogRead(JOYSTICK_PIN);
        float pwmDutyCycle = mapfloat(joystickValue, 0.0, 1023.0, 50.0, 90.0);
        setFanSpeed(pwmDutyCycle);
    }
    timeb = millis();
    Serial.print("processing time: ");
    Serial.println(timeb - timea);
    delay(10);
}
