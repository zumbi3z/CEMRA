#include <Wire.h>
#include "Magabot.h"

#include <ros.h>
#include <magabot/Status.h>
#include <magabot/Control.h>

#define N_SONARS 5
#define N_BUMPERS 4
#define N_IR 3
#define N_ENCODERS 2
#define PUB_FREQUENCY 20

void callback_actuation(const magabot::Control &msg);
void update_status(magabot::Status *status_msg);

Magabot robot;

unsigned long period = 100;
unsigned long next = 0;

int encoders[N_ENCODERS] = {0, 0};

ros::NodeHandle nh;
magabot::Status status_msg;
ros::Publisher status_pub("/magabot/status", &status_msg);
ros::Subscriber<magabot::Control> actuation_sub("/magabot/control", callback_actuation);

long publisher_timer;

void callback_actuation(const magabot::Control &control_msg)
{
	if(control_msg.set_velocity) { 
		robot.actuateMotors(control_msg.velocity.left_wheel, control_msg.velocity.right_wheel);
		analogWrite(3, 200);
	}

	if(control_msg.set_leds)
		robot.actuateLEDs(control_msg.leds.red, control_msg.leds.green, control_msg.leds.blue);
}

void update_status(magabot::Status *status_msg)
{
	robot.readBattery();
	status_msg->battery = robot.batteryRead;

	robot.readSonars();
	for(int i = 0; i < N_SONARS; i++)
	    status_msg->sonars[i] = robot.sonarRead[i].read();

	robot.readBumpers();
	for(int i = 0; i < N_BUMPERS; i++)
	    status_msg->bumpers[i] = robot.bumperRead[i];

	robot.readIR();
	for(int i = 0; i < N_IR; i++)
		status_msg->ir[i] = robot.irRead[i];

	robot.readClicks();
	status_msg->encoders.left_wheel = -robot.leftClicks;
	status_msg->encoders.right_wheel = robot.rightClicks;

	status_msg->timestamp = nh.now();

}

void setup() 
{
    nh.getHardware()->setBaud(115200); //or what ever baud you want
	nh.initNode();
	nh.subscribe(actuation_sub);
	nh.advertise(status_pub);

	next = millis();
}

void loop() 
{
        
	// if (millis() >= next) 
	{
	update_status(&status_msg);
    
    	status_pub.publish(&status_msg);

    	// next += period;

		// digitalWrite(13, !digitalRead(13));	
	}

	robot.update();
  
        //delay(100);
  
	nh.spinOnce();
        
        /*if (millis() >= next) {
          update_status(&status_msg);
          status_pub.publish(&status_msg);
          next += period;
          nh.spinOnce();
        }*/
  //robot.update();
}
