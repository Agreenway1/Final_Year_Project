#include <SimpleFOC.h>

// Motor and driver settings
#define MOTOR_POLE_PAIRS 7   // Adjust based on motor specs
#define VOLTAGE_LIMIT 6.0    // Max voltage to motor
#define VELOCITY_LIMIT 20.0  // Max velocity

// Define GPIO Pins for Pathology Input (from C2000)
#define PATHOLOGY_PIN_0 7  // Least Significant Bit (LSB)
#define PATHOLOGY_PIN_1 8  
#define PATHOLOGY_PIN_2 9  
#define PATHOLOGY_PIN_3 10 // Most Significant Bit (MSB)

// Define motor driver pins (SimpleFOCShield V2.0.3)
#define PWM_A 9
#define PWM_B 5
#define PWM_C 6
#define ENABLE 8

// Hall sensor pins
#define HALL_A 2
#define HALL_B 3
#define HALL_C 4

// Create BLDC motor instance
BLDCMotor motor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver(PWM_A, PWM_B, PWM_C, ENABLE);
HallSensor sensor(HALL_A, HALL_B, HALL_C, MOTOR_POLE_PAIRS);

// Define PID controller
PIDController velocityPID(0.5, 0.01, 0.001, 100);

// Default motor speed
float target_velocity = 0;

// Pathology-based PID settings
struct Pathology {
    int id;
    const char* name;
    float kp;  // Proportional gain
    float ki;  // Integral gain
    float kd;  // Derivative gain
} pathologies[] = {
    {1, "Appendicitis", 1.5, 0.1, 0.05},
    {2, "Ovarian Cyst (L)", 0.8, 0.05, 0.02},
    {3, "Ovarian Cyst (R)", 0.8, 0.05, 0.02},
    {4, "Hernia (R)", 1.2, 0.08, 0.04},
    {5, "Hernia (L)", 1.2, 0.08, 0.04},
    {6, "Diverticulitis", 2.0, 0.15, 0.08},
    {7, "Cecum (Inflamed)", 1.0, 0.07, 0.03},
    {8, "Sigmoid Colon (Inflamed)", 1.1, 0.08, 0.04},
    {9, "Ascending Colon (Inflamed)", 1.0, 0.06, 0.03},
    {10, "Stomach Ulcer", 1.3, 0.09, 0.05},
    {11, "Gallbladder Inflammation", 1.5, 0.1, 0.06},
    {12, "Pancreatitis", 2.5, 0.2, 0.1},
    {13, "Liver Enlargement", 1.2, 0.08, 0.04},
    {14, "Spleen Enlargement", 1.0, 0.07, 0.03},
    {15, "Mesenteric Adenitis", 1.1, 0.08, 0.04},
    {16, "Bladder Distension", 0.9, 0.06, 0.03}
};

void setup() {
    Serial.begin(115200);

    // Initialize Hall sensor
    sensor.init();
    motor.linkSensor(&sensor);

    // Configure motor driver
    driver.voltage_power_supply = 12; // 12V system
    driver.voltage_limit = VOLTAGE_LIMIT;
    driver.init();
    motor.linkDriver(&driver);

    // Configure motor controller
    motor.controller = MotionControlType::velocity;
    motor.PID_velocity = velocityPID;
    motor.voltage_limit = VOLTAGE_LIMIT;
    motor.velocity_limit = VELOCITY_LIMIT;

    // Initialize motor
    motor.init();
    motor.initFOC();

    // Set pathology input pins as INPUT
    pinMode(PATHOLOGY_PIN_0, INPUT);
    pinMode(PATHOLOGY_PIN_1, INPUT);
    pinMode(PATHOLOGY_PIN_2, INPUT);
    pinMode(PATHOLOGY_PIN_3, INPUT);

    Serial.println("BLDC Motor Ready. Waiting for pathology ID from GPIO...");
}

void loop() {
    motor.loopFOC();

    // Read Pathology ID from GPIO
    int pathology_id = readPathologyID();
    updatePID(pathology_id);

    // Apply target velocity (default for now)
    motor.move(target_velocity);

    // Debugging info
    Serial.print("Pathology ID: ");
    Serial.print(pathology_id);
    Serial.print(" - ");
    Serial.println(pathologies[pathology_id - 1].name);
}

// Read Pathology ID from GPIO Pins (4-bit binary input)
int readPathologyID() {
    int id = 0;
    id |= digitalRead(PATHOLOGY_PIN_0) << 0;  // Bit 0
    id |= digitalRead(PATHOLOGY_PIN_1) << 1;  // Bit 1
    id |= digitalRead(PATHOLOGY_PIN_2) << 2;  // Bit 2
    id |= digitalRead(PATHOLOGY_PIN_3) << 3;  // Bit 3

    // Ensure pathology ID is within range (1 to 16)
    if (id < 1 || id > 16) {
        return 1;  // Default to Appendicitis if invalid ID
    }

    return id;
}

// Update PID values based on pathology ID
void updatePID(int pathology_id) {
    for (int i = 0; i < sizeof(pathologies) / sizeof(Pathology); i++) {
        if (pathologies[i].id == pathology_id) {
            motor.PID_velocity.P = pathologies[i].kp;
            motor.PID_velocity.I = pathologies[i].ki;
            motor.PID_velocity.D = pathologies[i].kd;

            Serial.print("Updated PID for ");
            Serial.print(pathologies[i].name);
            Serial.print(": P=");
            Serial.print(pathologies[i].kp);
            Serial.print(", I=");
            Serial.print(pathologies[i].ki);
            Serial.print(", D=");
            Serial.println(pathologies[i].kd);
            return;
        }
    }

    Serial.println("Invalid pathology ID received, keeping default PID.");
}
