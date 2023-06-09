#include <Arduino.h>
#include <Servo.h>
/* Robot Behaviour Settings */

//#define WHACKY_ROBOT //uncomment if shity robot is used 

#ifdef WHACKY_ROBOT
#  define LEFT_STATIONARY  1460 // These may be incorrect and need to be retuned again.
#  define RIGHT_STATIONARY 1510
#  define ROTATION_TIME 900.0f
#  define FORWARD_CONST 100.0f
#  define STEP_TIME 10.0 // milliseconds
#  define GRIPPER_CLOSE 0
#  define GRIPPER_OPEN 80
#  define GRIPPER_UP 180
#else
#  define LEFT_STATIONARY  1500
#  define RIGHT_STATIONARY 1500
#  define ROTATION_TIME 1300.0f
#  define FORWARD_CONST 100.0f
#  define STEP_TIME 10.0 // milliseconds
#  define GRIPPER_CLOSE 70
#  define GRIPPER_OPEN 170
#  define GRIPPER_UP 0
#endif

const float FOV = 100.0f;

// TODO: Fine tune this, and the rest should work since angle-time relation should be linear.
const float ROTATION_CONST = ROTATION_TIME / 90.0f;

#define SWEEP_COUNT 10

/* End Robot Behaviour Settings */

// Pins
const int left_servo_pin         = 12;
const int right_servo_pin        = 13;
const int ultrasound_servo_pin   = 11;
const int claw_servo_pin         = 10;
const int ultrasound_pin_top     = 9;
const int ultrasound_pin_bottom  = 5;
const int left_wheel_sensor_pin  = 8;
const int right_wheel_sensor_pin = 7;
//Analog pins
const int IR_sensor_pin1= A0;
const int IR_sensor_pin2= A1;
const int IR_sensor_pin3= A2;

// Per ultrasound sensor data structure. Holds an array of distances found by sweeping.
struct UltrasoundSensor
{
    int pin;

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
void IR_sensor_Front_scan();
void IR_sensor_3_scan();
float get_ultrasound_distance_cm();
void  sweep_ultrasound();
float find_closest_distance(UltrasoundSensor *sensor, float *angle);
float find_furthest_distance(UltrasoundSensor *sensor, float *angle);

// Movement
void forward(int time);
void backward(int time);
void turnLeft(int time);
void turnRight(int time);
void halt_movement();

void forward_cm(float cm);
void turn_degrees(float theta);

void go_home();

// Other controls
void gripper_control(Gripper *status, int grip);//, int updown

/////////////////////////////////////////

// Sensor data
static UltrasoundSensor ultrasound_sensor_top;
static UltrasoundSensor ultrasound_sensor_bottom;
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
int IR_flag1=0; //cliff o, int updown
int Ultrasound_flag=0;
int Ultrasound_angle_flag=0;
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//END OF DECLARES 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
    Serial.begin(115200); //PIO monitoring budrate (needs to be set in platformio.ini as well)

    left_servo.attach(left_servo_pin); //Attach servos to pins
    right_servo.attach(right_servo_pin);
    gripperServo.attach(claw_servo_pin);
    
    ultrasound_servo.attach(ultrasound_servo_pin);//Attach ultrasound servo to pin

    pinMode(left_wheel_sensor_pin, INPUT);
    pinMode(right_wheel_sensor_pin, INPUT);
  

    ultrasound_sensor_top    = {.pin = ultrasound_pin_top};
    ultrasound_sensor_bottom = {.pin = ultrasound_pin_bottom};

    gripper_state = {.engaged = 0, .up = 0}; // TODO: It is already initialized to zero because static global.


    left_servo.writeMicroseconds(LEFT_STATIONARY);//write static position to servos
    right_servo.writeMicroseconds(RIGHT_STATIONARY);
    gripperServo.write(GRIPPER_OPEN); // Lift gripper
}


void loop()
{
#if 1
    delay(1000);
    forward_cm(10);
    delay(2000);
    forward_cm(20);
#endif
    
#if 0 // CALIBRATION TEST
    delay(1000);
    turn_degrees(90);
    delay(1000);
    turn_degrees(180);
    delay(1000);
    turn_degrees(360);
#endif
    
#if 0
    // Reset flags every loop. Makes sense to do this to me right now, but can go to if-elses if need be.
    flags = {};
    IR_sensor_BoundaryIR_scan();
    sweep_ultrasound();

    float closest_angle;
    float furthest_angle;
    float closest_distance = find_closest_distance(&ultrasound_sensor_top, &closest_angle);
    float furthest_distance = find_furthest_distance(&ultrasound_sensor_top, &furthest_angle);

    float threshold = 10.0f;
    if (closest_distance < threshold)
    {
        flags.Ultrasound = true;
    }

    
    bool boundary = flags.BoundaryIR;
    bool mountain = flags.FrontIR && flags.Ultrasound;

    bool Sample = flags.Ultrasound;

    char buffer[256];
    sprintf(buffer, "closest distance: %d cm @ angle %d", (int)closest_distance,
            (int)closest_angle);

    Serial.println(buffer);
    

    if (boundary)
    {
        Serial.println("Boundary detected.");
        turn_degrees(furthest_angle);
    }

    else if (mountain)
    {
        Serial.println("Mountain detected.");
    }

    else if (Sample)
    {
        turn_degrees(-closest_angle);
        forward(1000);
    } else{
        //forward(500);
        gripperServo.write(1700);
        gripper_control(gripper_state,1,1);
    }
#endif
    delay(1000);
    gripper_control(&gripper_state,1);
    delay(1000);
    gripper_control(&gripper_state,2);
    delay(1000);
    gripper_control(&gripper_state,0);
}

/////////////////////////////////////////

/*
  Left wheel counterclockwise  -> left_servo.writeMicroseconds(1700);
  Left wheel clockwise         -> left_servo.writeMicroseconds(1300);
  Right wheel counterclockwise -> right_servo.writeMicroseconds(1300);
  Right wheel clockwise        -> right_servo.writeMicroseconds(1700);
*/
////////////////////////////////
//PE
////////////////////////////////
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

float get_ultrasound_distance_cm(UltrasoundSensor *sensor)
{
    long duration;
    float cm;
    int pin = sensor->pin;

    pinMode(pin, OUTPUT);

    digitalWrite(pin, LOW);
    delayMicroseconds(2);
    digitalWrite(pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin, LOW);

    pinMode(pin, INPUT);
    duration = pulseIn(pin, HIGH);

    cm = duration / 29.0f / 2.0f;
    return cm;
}

void sweep_ultrasound()
{
    // This whole thing is bad, but we only have two ultrasound sensors.

    for (int i = 0; i < SWEEP_COUNT; i++)
    {
        ultrasound_sensor_top.distances[i] = get_ultrasound_distance_cm(&ultrasound_sensor_top);
        ultrasound_sensor_bottom.distances[i] = get_ultrasound_distance_cm(&ultrasound_sensor_bottom);
        float target_angle = (90.0f - FOV / 2.0f) + (i * FOV / SWEEP_COUNT);

        ultrasound_servo.write((int)target_angle);

        delay(100); // TODO: This delay and the one a couple lines down need to be fine-tuned.
    }

    for (int i = 0; i < SWEEP_COUNT; i++)
    {
        float t = (-FOV / 2.0f) + (i * FOV / SWEEP_COUNT);
        ultrasound_sensor_top.angles[i] = t;
        ultrasound_sensor_bottom.angles[i] = t;
    }

    ultrasound_servo.write(90 - (int)FOV / 2);
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

    *angle = sensor->angles[furthest_index];
    
    return furthest;
}

void IR_sensor_Boundary_scan()
{
    //IR sensor code
    //if cliff or boundary detected
    int IR_sensor1_value = analogRead(IR_sensor_pin1);
    //Serial.println(IR_sensor1_value);
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
    //Serial.println(IR_sensor2_value);
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
    //Serial.println(IR_sensor3_value);
    if(IR_sensor3_value==0){
        flags.TowerIR = 1;
    }else{
        flags.TowerIR = 0;
    }
}

void gripper_control(Gripper *status, int grip)//int updown
{
    //Servo called: gripperServo

    if(status->engaged != 1 && grip == 1){
        gripperServo.write(GRIPPER_CLOSE);
        delay(5000);
        status->engaged=1;
    }else if(status->engaged==0 && grip == 0){
        Serial.println("wrong input 1");
    }else if(status->engaged == 1 && grip == 1){
        Serial.println("wrong input 2");
    }else if(status->engaged!=0 && grip == 0){
        gripperServo.write(GRIPPER_OPEN);
        delay(5000);
        status->engaged=0;
    }else if (status->engaged != 2 && grip == 2){
        gripperServo.write(GRIPPER_UP);
        delay(5000);
        status->engaged=2;
    }else{
        Serial.println("wrong input 3");
    }
    
    #if 0
    if(status.up == 0 && updown == 1){
        gripperServo.write(GRIPPER_UP);
        delay(2000);
        //gripperServo.write(1500);
    }else if(status.up == 1 && updown == 0){ //FIXXXXXXXXXX ???????
        gripperServo.write(GRIPPER_UP); //SHOULD BE GRIPPER_DOWN
        delay(2000);
        //gripperServo.write(1500);
    }
    #endif
}

void safe_forward(float time)
{
    float step_count = time / STEP_TIME;
    for (int i = 0; i < step_count; i++)
    {
        IR_sensor_Boundary_scan();
        if (flags.BoundaryIR)
        {
            turn_degrees(90.0f);
            return;
        }
        
        forward(STEP_TIME);
    }

    
}

void forward_cm(float cm)
{
    safe_forward(cm * FORWARD_CONST);
}

void go_home()
{
    while(1)
    {
        sweep_ultrasound();
        float closest_angle;
        float furthest_angle;
        float closest_distance = find_closest_distance(&ultrasound_sensor_top, &closest_angle);
        float furthest_distance = find_furthest_distance(&ultrasound_sensor_top, &furthest_angle);

        if (closest_distance < 20.0f)
        {
            if (furthest_distance < 10.0f)
            {
                turn_degrees(furthest_angle);
            }
            else
            {
                static int prefered_dir = 0;
                if (prefered_dir == 0)
                {
                    turn_degrees(45);
                    prefered_dir = 1;
                }
                else
                {
                    turn_degrees(-45);
                    prefered_dir = 0;
                }
            }
            
            forward_cm(closest_distance); // Keep it safe by going the closest.
        }
        
        
    }
}
