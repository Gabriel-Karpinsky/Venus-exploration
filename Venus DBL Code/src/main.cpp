#include <Arduino.h>
#include <Servo.h>
/* Robot Behaviour Settings */

#define WHACKY_ROBOT

#ifdef WHACKY_ROBOT
#  define LEFT_STATIONARY  1460 // These may be incorrect and need to be retuned again.
#  define RIGHT_STATIONARY 1510
#  define ROTATION_TIME 900.0f
#  define GRIPPER_CLOSE 0
#  define GRIPPER_OPEN 80
#  define GRIPPER_UP 180
#else
#  define LEFT_STATIONARY  1500
#  define RIGHT_STATIONARY 1500
#  define ROTATION_TIME 1300.0f
#  define GRIPPER_CLOSE 70
#  define GRIPPER_OPEN 170
#  define GRIPPER_UP 0
#endif

// TODO: Fine tune this, and the rest should work since angle-time relation should be linear.
const float ROTATION_CONST = ROTATION_TIME / 90.0f;

#define SWEEP_COUNT 10

/* End Robot Behaviour Settings */

// Pins
const int left_servo_pin         = 12;
const int right_servo_pin        = 13;
const int ultrasound_servo_pin   = 11;
const int claw_servo_pin         = 10;
const int ultrasound_pin         = 9;
const int left_wheel_sensor_pin  = 8;
const int right_wheel_sensor_pin = 7;
//Analog pins
const int IR_sensor_pin1= A0;
const int IR_sensor_pin2= A1;
const int IR_sensor_pin3= A2;
////////////////////////////////////////////////////////

// Per ultrasound sensor data structure. Holds an array of distances found by sweeping.
struct UltrasoundSensor
{
    float fov;

    float distances[SWEEP_COUNT]; // Sweep right to left
    float angles[SWEEP_COUNT];
};

struct Gripper //🦶🍆😜💦💀 Zamn girl show me them grippers 
{
    int engaged;
    int up;
};
/////////////////////////////////////////

// Components
Servo left_servo;
Servo right_servo;
Servo ultrasound_servo;
Servo gripperServo;

/////////////////////////////////////////

/* forward declarations */
void  IR_sensor_BoundaryIR_scan();
float get_ultrasound_distance_cm();
void  sweep_ultrasound(UltrasoundSensor *state);
float find_closest_distance(UltrasoundSensor *state, float *angle);
float find_furthest_distance(UltrasoundSensor *sensor, float *angle);

// Movement
void forward(int time);
void backward(int time);
void turnLeft(int time);
void turnRight(int time);
void halt_movement();

void turn_degrees(float theta);

// Other controls
void gripper_control(Gripper status, int grip, int updown);

/////////////////////////////////////////

// Sensor data
static UltrasoundSensor ultrasound_sensor;
static Gripper gripper_state;


// Decision making flags
struct Flags
{
    bool BoundaryIR; // Boundary or cliff
    bool FrontIR;    // Mountain or sample
    bool TowerIR;    // Tower

    bool Ultrasound; // Set if passes threshold
};

static Flags flags;

// TODO: Clarification still needed on flags.
#if 0
int IR_flag1=0; //cliff or boundary
int IR_flag2=0; //mountain or sample
int IR_flag3=0; //Tower

int Ultrasound_flag=0;
int Ultrasound_angle_flag=0;
#endif

/////////////////////////////////////////


void setup()
{
    Serial.begin(115200); //PIO monitoring budrate (needs to be set in platformio.ini as well)

    left_servo.attach(left_servo_pin);
    right_servo.attach(right_servo_pin);
    
    ultrasound_servo.attach(ultrasound_servo_pin);

    pinMode(left_wheel_sensor_pin, INPUT);
    pinMode(right_wheel_sensor_pin, INPUT);

    ultrasound_sensor = {.fov = 90.0f};
    gripper_state     = {.engaged = 0, .up = 0}; // TODO: It is already initialized to zero because static global.


    left_servo.writeMicroseconds(LEFT_STATIONARY);
    right_servo.writeMicroseconds(RIGHT_STATIONARY);
    gripperServo.write(GRIPPER_UP);
}


void loop()
{
#if 1
    // Reset flags every loop. Makes sense to do this to me right now, but can go to if-elses if need be.
    flags = {};
    IR_sensor_BoundaryIR_scan();
    sweep_ultrasound(&ultrasound_sensor);

    float closest_angle;
    float closest_distance = find_closest_distance(&ultrasound_sensor, &closest_angle);

    float threshold = 15.0f;
    if (closest_distance < threshold)
    {
        flags.Ultrasound = true;
    }

    
    bool boundary = flags.BoundaryIR;
    bool mountain = flags.FrontIR && flags.Ultrasound;

    bool just_ultrasound = flags.Ultrasound;

    char buffer[256];
    sprintf(buffer, "%d cm @ angle %d", (int)closest_distance,
            (int)closest_angle);

    Serial.println(buffer);
    

    if (boundary)
    {
        Serial.println("Boundary detected.");
        turn_degrees(30);
    }

    else if (mountain)
    {
        Serial.println("Mountain detected.");
    }

    else if (just_ultrasound)
    {
        turn_degrees(-closest_angle);
        forward(3000);
    } else{
        forward(500);
    }
#endif
}

/////////////////////////////////////////

/*
  Left wheel counterclockwise  -> left_servo.writeMicroseconds(1700);
  Left wheel clockwise         -> left_servo.writeMicroseconds(1300);
  Right wheel counterclockwise -> right_servo.writeMicroseconds(1300);
  Right wheel clockwise        -> right_servo.writeMicroseconds(1700);
*/
void forward(int time)
{
    left_servo.writeMicroseconds(1700);
    right_servo.writeMicroseconds(1300);
    delay(time);
    halt_movement();
}

void backward(int time)
{
    left_servo.writeMicroseconds(1300);
    right_servo.writeMicroseconds(1700);
    delay(time);
    halt_movement();
}

void turnLeft(int time)
{
    left_servo.writeMicroseconds(1300);
    right_servo.writeMicroseconds(1300);
    delay(time);
    halt_movement();
}

void turnRight(int time)
{
    left_servo.writeMicroseconds(1700);
    right_servo.writeMicroseconds(1700);
    delay(time);
    halt_movement();
}

void halt_movement()
{
    left_servo.writeMicroseconds(LEFT_STATIONARY);
    right_servo.writeMicroseconds(RIGHT_STATIONARY);
}

// Positive angle is anti-clockwise looking top-down
void turn_degrees(float theta)
{
    halt_movement();
    
    if (theta > 0)
    {
        turnLeft(theta * ROTATION_CONST);
    }
    else if (theta < 0)
    {
        theta = -theta;
        turnRight(theta * ROTATION_CONST);
    }
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

void sweep_ultrasound(UltrasoundSensor *state)
{
    float fov = state->fov;

    for (int i = 0; i < SWEEP_COUNT; i++)
    {
        float distance = get_ultrasound_distance_cm();
        state->distances[i] = distance;

        float target_angle = (90.0f - fov / 2.0f) + (i * fov / SWEEP_COUNT);

        ultrasound_servo.write((int)target_angle);

        delay(100); // TODO: This delay and the one a couple lines down need to be fine-tuned.
    }

    for (int i = 0; i < SWEEP_COUNT; i++)
    {
        float t = (-fov / 2.0f) + (i * fov / SWEEP_COUNT);
        state->angles[i] = t;
    }

    ultrasound_servo.write(90 - (int)fov / 2);
    delay(700); // Delay so that the ultrasound servo has enough time to go back to the start position.
}

// Returns the angle at which this distance was sampled through second parameter.
float find_closest_distance(UltrasoundSensor *sensor, float *angle)
{
    float closest = 10000.0f; // Just some large number.
    int closest_index = -1;

    for (int i = 0; i < SWEEP_COUNT; i++)
    {
        if (sensor->distances[i] < closest)
        {
            closest = sensor->distances[i];
            closest_index = i;
        }
    }

    // assert(closest_index > 0);

    *angle = sensor->angles[closest_index];
    
    return closest;
}

// Returns the angle at which this distance was sampled through second parameter.
float find_furthest_distance(UltrasoundSensor *sensor, float *angle)
{
    float furthest = 0.0f;
    int furthest_index = -1;

    for (int i = 0; i < SWEEP_COUNT; i++)
    {
        if (sensor->distances[i] > furthest)
        {
            furthest = sensor->distances[i];
            furthest_index = i;
        }
    }

    // assert(closest_index > 0);

    *angle = sensor->angles[furthest_index];
    
    return furthest;
}

void IR_sensor_BoundaryIR_scan()
{
    //IR sensor code
    //if cliff or boundary detected
    int IR_sensor1_value = analogRead(IR_sensor_pin1);
    Serial.println(IR_sensor1_value);
    if(IR_sensor1_value==0){
        flags.BoundaryIR = 1;
    }else{
        flags.BoundaryIR = 0;
    }
}
void IR_sensor_Front_scan()
{
    //IR sensor code
    //if object to the front detected
    int IR_sensor2_value = analogRead(IR_sensor_pin2);
    Serial.println(IR_sensor2_value);
    if(IR_sensor2_value==0){
        flags.FrontIR = 1;
    }else{
        flags.FrontIR = 0;
    }
}
void IR_sensor_3_scan()
{
    //IR sensor code
    //if Tower detected
    int IR_sensor3_value = analogRead(IR_sensor_pin3);
    Serial.println(IR_sensor3_value);
    if(IR_sensor3_value==0){
        flags.TowerIR = 1;
    }else{
        flags.TowerIR = 0;
    }
}

void gripper_control(Gripper status, int grip, int updown)
{
    //Servo called: gripperServo

    if(status.engaged == 0 && grip == 1){
        gripperServo.write(GRIPPER_CLOSE);
        delay(2000);
    }else if(status.engaged==1 && grip == 0){
        gripperServo.write(GRIPPER_OPEN);
        delay(2000);
    }

    if(status.up == 0 && updown == 1){
        gripperServo.write(GRIPPER_UP);
        delay(2000);
    }else if(status.up == 1 && updown == 0){ //FIXXXXXXXXXX ???????
        gripperServo.write(GRIPPER_UP); //SHOULD BE GRIPPER_DOWN
        delay(2000);
    }
}
