#include <Arduino.h>
#include "pwmgen.h"

// 2 PWM generator for 2 motor
static pwm_gen pwmA; // chan1 -> GPIO27
static pwm_gen pwmB; // chan2 -> GPIO17

// ====== DIR theo sơ đồ bạn ======
static const gpio_num_t A_IN1 = GPIO_NUM_23; // DIR_11 -> AI1
static const gpio_num_t A_IN2 = GPIO_NUM_18; // DIR_12 -> AI2

static const gpio_num_t B_IN1 = GPIO_NUM_13; // DIR_21 -> BI1
static const gpio_num_t B_IN2 = GPIO_NUM_16; // DIR_22 -> BI2

static void motorA_dir(bool forward) {
  digitalWrite(A_IN1, forward ? HIGH : LOW);
  digitalWrite(A_IN2, forward ? LOW  : HIGH);
}
static void motorB_dir(bool forward) {
  digitalWrite(B_IN1, forward ? HIGH : LOW);
  digitalWrite(B_IN2, forward ? LOW  : HIGH);
}

static void motorA_brake() { digitalWrite(A_IN1, HIGH); digitalWrite(A_IN2, HIGH); }
static void motorB_brake() { digitalWrite(B_IN1, HIGH); digitalWrite(B_IN2, HIGH); }

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(A_IN1, OUTPUT); pinMode(A_IN2, OUTPUT);
  pinMode(B_IN1, OUTPUT); pinMode(B_IN2, OUTPUT);

  // STBY is hardwired to 3V3, so no need to set

  motorA_brake();
  motorB_brake();

  // PWM: 20kHz, duty in % (0..100)
  pwmA.begin(1, 20000, 0);  // chan1 -> GPIO27
  pwmB.begin(2, 20000, 0);  // chan2 -> GPIO17
  pwmA.start_pwm();
  pwmB.start_pwm();

  Serial.println("MCPWM open-loop test (2 motors): sweep duty FWD/REV.");
}

static void sweep_both(bool forward, float maxDuty=60.0f) {
  motorA_dir(forward);
  motorB_dir(forward);

  for (float d = 0; d <= maxDuty; d += 5) {
    pwmA.set_duty(d);
    pwmB.set_duty(d);
    Serial.printf("%s duty=%.1f%%\n", forward ? "FWD" : "REV", d);
    delay(500);
  }

  pwmA.set_duty(0);
  pwmB.set_duty(0);
  delay(500);
}

void loop() {
  sweep_both(true, 60);
  sweep_both(false, 60);

  motorA_brake();
  motorB_brake();
  delay(1500);
}
