/*
 * Simple DRV8833 Motor Test for XIAO ESP32S3
 * Spins both motors forward at 75% duty cycle.
 */

// TOUCH1_GPIO1_A0_D0 is connected to AIN1
// TOUCH2_GPIO2_A1_D1 is connected to AIN2
// TOUCH3_GPIO3_A2_D2 is connected to BIN2
// TOUCH4_GPIO4_A3_D3 is connected to BIN1

// Left Motor (Channel A)
const int AIN1 = 1; // D0
const int AIN2 = 2; // D1

// Right Motor (Channel B)
const int BIN1 = 4; // D3
const int BIN2 = 3; // D2

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
}

void loop() {
  // Motor Speed (0 - 255)
  int motorSpeed = 190; // ~75% duty cycle

  // --- Spin Left Motor Forward ---
  analogWrite(AIN1, motorSpeed);
  analogWrite(AIN2, 0);

  // --- Spin Right Motor Forward ---
  // BIN1 is connected to D3, BIN2 to D2
  analogWrite(BIN1, motorSpeed);
  analogWrite(BIN2, 0);

  delay(2000);

  // --- Full Stop ---
  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);

  delay(1000);
}
