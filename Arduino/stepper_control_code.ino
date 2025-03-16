#include <AccelStepper.h>
#include <Wire.h>

// Serial Communication with TI C2000
#define SERIAL_BAUD 115200

// Stepper Motor Control Pins (CNC Shield)
#define X_STEP_PIN 2
#define X_DIR_PIN 5
#define X_ENABLE_PIN 8

#define Y_STEP_PIN 3
#define Y_DIR_PIN 6
#define Y_ENABLE_PIN 9

// Stepper Motor Voltage Feedback (Analog Inputs)
#define X_VOLTAGE_SENSOR A0
#define Y_VOLTAGE_SENSOR A1

// Motor Constants
#define STEPS_PER_MM 80   // Adjust based on stepper motor & driver settings
#define MAX_SPEED 5000    // Max speed in steps/sec
#define MAX_ACCELERATION 1000 // Acceleration

// MPC Control Parameters
double x_target = 0, y_target = 0;  // Target positions from C2000
double x_current = 0, y_current = 0;  // Current positions estimated from stepper voltage
double u_x = 0, u_y = 0;  // MPC control outputs

// Define Stepper Motors using AccelStepper Library
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

// Function Prototypes
void read_target_position();
void read_current_position();
void run_mpc_control();
void enable_steppers(bool enable);

void setup() {
    Serial.begin(SERIAL_BAUD);
    
    // Initialize Stepper Motors
    stepperX.setMaxSpeed(MAX_SPEED);
    stepperX.setAcceleration(MAX_ACCELERATION);
    stepperY.setMaxSpeed(MAX_SPEED);
    stepperY.setAcceleration(MAX_ACCELERATION);

    // Set Enable Pins as OUTPUT and enable steppers
    pinMode(X_ENABLE_PIN, OUTPUT);
    pinMode(Y_ENABLE_PIN, OUTPUT);
    enable_steppers(true); // Enable stepper drivers
}

void loop() {
    read_target_position();  // Read target (x, y) position from C2000
    read_current_position(); // Read current position from stepper driver voltage
    run_mpc_control();       // Run Model Predictive Control (MPC)

    // Apply calculated control inputs to stepper motors
    stepperX.moveTo(x_current + u_x);
    stepperY.moveTo(y_current + u_y);

    stepperX.run();
    stepperY.run();

    delay(10);  // Small delay to avoid excessive computation
}

// Read target (x, y) positions from C2000 over Serial (UART)
void read_target_position() {
    if (Serial.available()) {
        String data = Serial.readStringUntil('\n');
        sscanf(data.c_str(), "%lf,%lf", &x_target, &y_target);
    }
}

// Read stepper driver voltage and estimate current position
void read_current_position() {
    double x_voltage = analogRead(X_VOLTAGE_SENSOR) * (5.0 / 1023.0);  // Convert to voltage
    double y_voltage = analogRead(Y_VOLTAGE_SENSOR) * (5.0 / 1023.0);

    // Convert voltage to position estimate (Calibration required)
    x_current = x_voltage * STEPS_PER_MM * 10;  // Example scaling factor
    y_current = y_voltage * STEPS_PER_MM * 10;
}

// Model Predictive Control (MPC) for Stepper Motor Control
void run_mpc_control() {
    // MPC Prediction Model (simplified)
    double error_x = x_target - x_current;
    double error_y = y_target - y_current;

    // Control Output Calculation (MPC optimization not fully implemented)
    u_x = constrain(error_x * 0.5, -MAX_SPEED, MAX_SPEED);  // Proportional control placeholder
    u_y = constrain(error_y * 0.5, -MAX_SPEED, MAX_SPEED);
}

// Function to Enable or Disable Stepper Motors
void enable_steppers(bool enable) {
    if (enable) {
        digitalWrite(X_ENABLE_PIN, LOW);  // LOW = Enable motor
        digitalWrite(Y_ENABLE_PIN, LOW);
    } else {
        digitalWrite(X_ENABLE_PIN, HIGH); // HIGH = Disable motor
        digitalWrite(Y_ENABLE_PIN, HIGH);
    }
}
