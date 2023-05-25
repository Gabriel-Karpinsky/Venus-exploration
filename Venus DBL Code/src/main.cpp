#include <Arduino.h>
#include <Servo.h>

#ifdef WHACKY_ROBOT
#  define LEFT_STATIONARY  1460  // These may be incorrect and need to be retuned again.
#  define RIGHT_STATIONARY 1520
#  define GRIPPER_CLOSE 0
#  define GRIPPER_OPEN 80
#  define GRIPPER_UP 180
#else
#  define LEFT_STATIONARY  1500
#  define RIGHT_STATIONARY 1500
#  define GRIPPER_CLOSE 70
#  define GRIPPER_OPEN 170
#  define GRIPPER_UP 0
#endif

#define SWEEP_COUNT 10

#define UINT16_MAX 65535

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
struct UltrasoundSensor
{
    float fov;

    float distances[SWEEP_COUNT]; // Sweep left to right
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

// Ultrasound
float get_ultrasound_distance_cm();
void  sweep_ultrasound(UltrasoundSensor *state);
int   find_closest_angle(UltrasoundSensor *state);


// Movement
void  forward(int time);
void  backward(int time);
void  turnLeft(int time);
void  turnRight(int time);
void  halt_movement();

void  turn_degrees(float theta);

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
    gripper_state     = {.engaged = 0, .up = 0};


    left_servo.writeMicroseconds(LEFT_STATIONARY);
    right_servo.writeMicroseconds(RIGHT_STATIONARY);

}

void loop()
{
    sweep_ultrasound(&ultrasound_sensor);

    
    bool boundary = flags.BoundaryIR;
    bool mountain = flags.BoundaryIR && flags.Ultrasound;

    if (boundary)
    {
        Serial.println("Boundary detected.");
        turn_degrees(90);
    }

    if (mountain)
    {
        Serial.println("Mountain detected.");
    }
    


    // TODO: #if 0'ed for now, because there could be a better way of doing this.
#if 0
    if(IR_flag2 && Ultrasound_flag)       // mountain detected
    {
        printf("mountain\n");
    }
    else if(IR_flag1 && !Ultrasound_flag) // cliff or boundary detected
    {
        printf("cliff or boundary\n");
        turnLeftAngle(45,100);
        halt_movement();
    }
    else if(!IR_flag2 && Ultrasound_flag) // sample detected
    {
        printf("sample\n");
        gripper_control(gripper_state,1,1);// close gripper and UP gripper
    }
    else
    {
        turnLeftAngle(45,1000);
        halt_movement();
    }
#endif
    
}


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
    // Fine tune this, and the rest should work since angle-time relation is linear.
    float rotation_constant = 1;

    halt_movement();
    
    if (theta > 0)
    {
        turnLeft(theta * rotation_constant);
    }
    else if (theta < 0)
    {
        theta = -theta;
        turnRight(theta * rotation_constant);
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

int find_closest_angle(UltrasoundSensor *state)
{
    uint16_t closest = UINT16_MAX;
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

void IR_sensor1_scan()
{
    // IR sensor code
    // if cliff or boundary detected
    printf("balls");
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
