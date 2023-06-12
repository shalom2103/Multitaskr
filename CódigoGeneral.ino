#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AS5600.h>
#include <SoftwareSerial.h>

// Define the PCA9685 driver
Adafruit_PWMServoDriver pca9685;

// Define the AS5600 encoder
AS5600 encoder1;
AS5600 encoder2;

// Define the servo channels
uint8_t servo1_channel = 0;
uint8_t servo2_channel = 1;

// Define the servo angle ranges
int servo1_min_angle = 0;
int servo1_max_angle = 180;
int servo2_min_angle = 0;
int servo2_max_angle = 360;

// Define the Bluetooth connection
SoftwareSerial bluetoothSerial(10, 11); // RX, TX

// Function to map a value from one range to another
int map_value(int value, int in_min, int in_max, int out_min, int out_max) {
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  // Start the serial communication
  Serial.begin(9600);
  
  // Start the I2C communication
  Wire.begin();
  
  // Initialize the PCA9685 driver
  pca9685.begin();
  pca9685.setPWMFreq(50); // Set the PWM frequency (you may need to adjust this)
  
  // Initialize the AS5600 encoders
  encoder1.begin();
  encoder2.begin();
  
  // Initialize the Bluetooth connection
  bluetoothSerial.begin(9600);
}

void loop() {
  // Check if there is Bluetooth data available
  if (bluetoothSerial.available()) {
    // Read the Bluetooth data
    char data = bluetoothSerial.read();
    
    // Process the Bluetooth data
    // Assuming each slider value is sent as a single character '0'-'9' representing a range of -100 to 100
    int servo1_position = map_value(data - '0', 0, 9, -100, 100);
    
    // Read the next character for the second servo
    while (!bluetoothSerial.available());
    data = bluetoothSerial.read();
    int servo2_position = map_value(data - '0', 0, 9, -100, 100);
    
    // Map the servo positions to angles
    int servo1_angle = map_value(servo1_position, -100, 100, servo1_min_angle, servo1_max_angle);
    int servo2_angle = map_value(servo2_position, -100, 100, servo2_min_angle, servo2_max_angle);
    
    // Set the servo angles using PCA9685
    pca9685.setPWM(servo1_channel, 0, servo1_angle);
    pca9685.setPWM(servo2_channel, 0, servo2_angle);
    
    // Print the servo angles for debugging
    Serial.print("Servo 1 angle: ");
    Serial.println(servo1_angle);
    Serial.print("Servo 2 angle: ");
    Serial.println(servo2_angle);
  }
}
