#if 0
#include <Arduino.h>
#include <Servo.h>
#include <Arduino_FreeRTOS.h>
#include "Thread.h"
#include "ThreadController.h"
#include <TimerOne.h>
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

// Create a new Class, called SensorThread, that inherits from Thread
class SensorThread: public Thread
{
public:
	int value;
	int pin;

	// No, "run" cannot be anything...
	// Because Thread uses the method "run" to run threads,
	// we MUST overload this method here. using anything other
	// than "run" will not work properly...
	void run(){
		// Reads the analog pin, and saves it localy
		value = map(analogRead(pin), 0,1023,0,255);
		runned();
	}
};

// Now, let's use our new class of Thread
SensorThread analog1 = SensorThread();
SensorThread analog2 = SensorThread();

// Instantiate a new ThreadController
ThreadController controller = ThreadController();

// This is the callback for the Timer
void timerCallback(){
	controller.run();
}

void setup(){

	Serial.begin(9600);

	// Configures Thread analog1
	analog1.pin = A1;
	analog1.setInterval(100);

	// Configures Thread analog2
	analog2.pin = A2;
	analog2.setInterval(100);

	// Add the Threads to our ThreadController
	controller.add(&analog1);
	controller.add(&analog2);

	Timer1.initialize(20000);
	Timer1.attachInterrupt(timerCallback);
	Timer1.start();

}

void loop(){
	// Do complex-crazy-timeconsuming-tasks here
	delay(1000);
	
	// Get the fresh readings
	Serial.print("Analog1 Thread: ");
	Serial.println(analog1.value);

	Serial.print("Analog2 Thread: ");
	Serial.println(analog2.value);

	// Do more complex-crazy-timeconsuming-tasks here
	delay(1000);

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

#endif