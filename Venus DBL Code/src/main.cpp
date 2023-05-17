#include <Arduino.h>
#include <Servo.h>

#ifdef WHACKY_ROBOT
#  define LEFT_STATIONARY  1460
#  define RIGHT_STATIONARY 1520
#else
#  define LEFT_STATIONARY  1500
#  define RIGHT_STATIONARY 1500
#endif

#define SWEEP_COUNT 10

// Pins
const int left_servo_pin         = 13;
const int right_servo_pin        = 12;
const int ultrasound_servo_pin   = 11;
const int claw_servo_pin         = 10;
const int ultrasound_pin         = 9;
const int left_wheel_sensor_pin  = 8;
const int right_wheel_sensor_pin = 7;


struct UltrasoundState
{
    float distance; // In centimeters.
    float angle;    // In degrees.
};


// Components
Servo left_servo;
Servo right_servo;
Servo ultrasound_servo;

UltrasoundState ultrasound_state;

int IR_flag = 0;
int Ultrasound_flag = 0;
int Ultrasound_angle_flag = 0;

// Ultrasound
void update_ultrasound_state(UltrasoundState *state);

// Movement
void move_to_free_space();

void setup()
{
    Serial.begin(115200);

    left_servo.attach(left_servo_pin);
    right_servo.attach(right_servo_pin);
    ultrasound_servo.attach(ultrasound_servo_pin);
}

void loop()
{
    move_to_free_space();
}

float get_ultrasound_distance_cm()
{
    long duration;
    float cm;
    
    pinMode(ultrasound_pin, OUTPUT);
    
    digitalWrite(ultrasound_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasound_pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(ultrasound_pin, LOW);
    
    pinMode(ultrasound_pin, INPUT);
    duration = pulseIn(ultrasound_pin, HIGH);
    
    cm = duration / 29.0f / 2.0f;
    return cm;
}

void update_ultrasound_state(UltrasoundState *state)
{
    state->distance = get_ultrasound_distance_cm();
    state->angle = 0;
}

void move_to_free_space()
{
    float fov = 90.0f;
    float distances[SWEEP_COUNT]; // Left to right

    for (int i = 0; i < SWEEP_COUNT; i++)
    {
        update_ultrasound_state(&ultrasound_state);
        distances[i] = ultrasound_state.distance;

        float target_angle = 45.0f + (i * fov / SWEEP_COUNT);
        
        char buffer[256];
        sprintf(buffer, "%d", (int)ultrasound_state.distance);
        Serial.println(buffer);
        
        ultrasound_servo.write((int)target_angle);

        delay(100);
    }
    
    ultrasound_servo.write(45);
    delay(500); // Delay so that the ultrasound servo has enough time to go back to the start position.
}
