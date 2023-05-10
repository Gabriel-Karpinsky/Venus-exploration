#include <Arduino.h>
#include <Servo.h>
#include <Arduino_FreeRTOS.h>

// Pins
const int left_servo_pin         = 13;
const int right_servo_pin        = 12;
const int ultrasound_servo_pin   = 11;
const int claw_servo_pin         = 10;
const int ultrasound_pin         = 9;
const int left_wheel_sensor_pin  = 8;
const int right_wheel_sensor_pin = 7;

// Components
Servo left_servo;
Servo right_servo;

// Inputs
uint32_t front_ultrasound_distance;

// Tasks (threads)
void thread_delay_millis(int milliseconds);
void TaskSensorReading(void *pvParameters);
void TaskMovement(void *pvParameters);

void setup()
{
    Serial.begin(115200);

    // Sets up tasks (threads). One for movement logic, one for sensor reading.
    xTaskCreate(TaskSensorReading,
                "SensorReading",
                128,         // Allocated stack size.
                NULL,
                2,           // Priority.
                NULL);

    xTaskCreate(TaskMovement,
                "Movement",
                128,
                NULL,
                1,
                NULL);

    left_servo.attach(left_servo_pin);
    right_servo.attach(right_servo_pin);

    left_servo.write(90);
    right_servo.write(90);
}

void loop()
{
    left_servo.writeMicroseconds(1500);
    right_servo.writeMicroseconds(1500);
}

void thread_delay_millis(int milliseconds)
{
    vTaskDelay(milliseconds / portTICK_PERIOD_MS);
}

void TaskSensorReading(void *pvParameters)
{
    (void) pvParameters;

    // Tasks require infinite loops inside.
    for (;;)
    {
        Serial.println("This should be printed out once a second.\n");
        thread_delay_millis(1000);

    }
}

void TaskMovement(void *pvParameters)
{
    (void) pvParameters;

    // Tasks require infinite loops inside.
    for (;;)
    {
        Serial.println("This should be printed out twice a second.\n");
        thread_delay_millis(2000);
    }
}

