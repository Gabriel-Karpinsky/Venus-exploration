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

#define INT16_MAX 65535

// Pins
const int left_servo_pin         = 13;
const int right_servo_pin        = 12;
const int ultrasound_servo_pin   = 11;
const int claw_servo_pin         = 10;
const int ultrasound_pin         = 9;
const int left_wheel_sensor_pin  = 8;
const int right_wheel_sensor_pin = 7;
////////////////////////////////////////////////////////

//Ultrasound struct declaration used to update ultrasound data globaly
struct UltrasoundState
{
    float fov;
    
    float distances[SWEEP_COUNT]; // Sweep left to right
    float angles[SWEEP_COUNT];
};
//////////////////////////////////////////////////////////

// Mounted servos

// Components
Servo left_servo;
Servo right_servo;
Servo ultrasound_servo;
//////////////////////////////////


// forward declarations
void update_ultrasound_state(UltrasoundState *state);
int  get_ultrasound_distance();
void turnLeftAngle(int angle, int time);
void move_to_free_space();

///////////////////////////////////////

// State
static UltrasoundState ultrasound_state;

// Decision making flags 
int IR_flag=0;
int Ultrasound_flag=0;
int Ultrasound_angle_flag=0;
//////////////////////////////


void setup()
{
    Serial.begin(115200); //PIO monitoring budrate (needs to be set in platformio.ini as well)

    left_servo.attach(left_servo_pin);
    right_servo.attach(right_servo_pin);
    ultrasound_servo.attach(ultrasound_servo_pin);
    
    pinMode(left_wheel_sensor_pin, INPUT);
    pinMode(right_wheel_sensor_pin, INPUT);

    ultrasound_state = {.fov = 90.0f};
    
    left_servo.writeMicroseconds(LEFT_STATIONARY);
    right_servo.writeMicroseconds(RIGHT_STATIONARY);

}


void forward(int time)                       // Forward function
{
  left_servo.writeMicroseconds(1700);         // Left wheel counterclockwise
  right_servo.writeMicroseconds(1300);        // Right wheel clockwise
  delay(time);                               // Maneuver for time ms
}

void turnLeftAngle(int angle, int time){
  left_servo.write(angle);      // Left wheel counterclockwise??? with given angle
  right_servo.write(angle);
          // Right wheel clockwise??? with given angle
  delay(time);       
}
void turnLeft(int time)                      // Left turn function
{
  left_servo.writeMicroseconds(1300);         // Left wheel clockwise
  right_servo.writeMicroseconds(1300);        // Right wheel clockwise
  delay(time);                               // Maneuver for time ms
}

void turnRight(int time)                     // Right turn function
{
  left_servo.writeMicroseconds(1700);         // Left wheel counterclockwise
  right_servo.writeMicroseconds(1700);        // Right wheel counterclockwise
  delay(time);                               // Maneuver for time ms
}

void backward(int time)                      // Backward function
{
  left_servo.writeMicroseconds(1300);         // Left wheel clockwise
  right_servo.writeMicroseconds(1700);        // Right wheel counterclockwise
  delay(time);                               // Maneuver for time ms
}
void wait(int time)
{
  left_servo.writeMicroseconds(1500);
  right_servo.writeMicroseconds(1500);
  delay(time);         // Left wheel clockwise
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

void sweep_ultrasound(UltrasoundState *state)
{
    float fov = state->fov;
    
    for (int i = 0; i < SWEEP_COUNT; i++)
    {
        float distance = get_ultrasound_distance_cm();
        state->distances[i] = distance;

        float target_angle = (90.0f - fov / 2.0f) + (i * fov / SWEEP_COUNT);
        
        char buffer[256];
        sprintf(buffer, "%d cm @ angle %d with FOV %d", (int)distance,
                (int)target_angle, (int)fov);
        
        Serial.println(buffer);
        
        ultrasound_servo.write((int)target_angle);
        
        delay(150);
    }

    for (int i = 0; i < SWEEP_COUNT; i++)
    {
        state->angles[i] = (90.0f - fov / 2.0f) + (i * fov / SWEEP_COUNT);
    }
    
    ultrasound_servo.write(90 - (int)fov / 2);
    delay(400); // Delay so that the ultrasound servo has enough time to go back to the start position.
}

int find_closest_angle(UltrasoundState *state)
{
    uint16_t closest = INT16_MAX;
    int closest_index = -1;
    
    for (int i = 0; i < SWEEP_COUNT; i++)
    {
        if (state->distances[i] < closest)
        {
            closest = state->distances[i];
            closest_index = i;
        }
    }
    
    return state->angles[closest_index];
}

// Positive angle is anti-clockwise
void turn_degrees(float theta)
{
    if (theta > 0)
    {
        turnRight(0);
    }
}

void loop()
{
    sweep_ultrasound(&ultrasound_state);
    find_closest_angle(&ultrasound_state);

    turn_degrees(90);
    delay(2000);
    
    if(IR_flag && Ultrasound_flag){//mountain detected
        printf("mountain");
    }else if(IR_flag && !Ultrasound_flag){//cliff or boundary detected
        turnLeftAngle(45,100);
        wait(100);
    }else if(!IR_flag && Ultrasound_flag){//sample detected
        printf("balls");
    }else{
//        turnLeftAngle(45,1000);
//        wait(5000);
    } 
}
