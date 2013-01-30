#include "WPILib.h"
#include <PWM.h>
#include <Joystick.h>
//#include <Jaguar.h>
/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */ 

#define NUM_BUTTONS (13)

class RobotDemo : public SimpleRobot
{
	RobotDrive myRobot; // robot drive system
	
	Joystick left_jstick; // only joystick
	Joystick right_jstick;
	
	Jaguar motor_shooter;
	
	Timer drive_timer;
	Timer timer;
	Timer trigger_timer;
	Timer timer_one_ms;
	
	Servo camera_yaw_servo;
	Servo camera_tilt_servo;
	
	static const float m_sensitivity = .25;
	static const int NUM_JOYSTICK_BUTTONS = 16;
	bool m_rightStickButtonState[(NUM_JOYSTICK_BUTTONS+1)];
	bool m_leftStickButtonState[(NUM_JOYSTICK_BUTTONS+1)];	


public:

	RobotDemo(void):
		myRobot(1, 2),	// these must be initialized in the same order
		left_jstick(1),		// as they are declared above.
		right_jstick(2),
		motor_shooter(3),
		drive_timer(),
		timer(),
		trigger_timer(),
		timer_one_ms(),
		camera_yaw_servo(4),
		camera_tilt_servo(5)
	{
		myRobot.SetExpiration(0.1);
	}

	/**
	 * Drive left & right motors for 2 seconds then stop
	 */

	void Autonomous(void)
	{
		cout<<"Here";
		myRobot.SetSafetyEnabled(false);
		myRobot.Drive(.2, 0.0); 	// drive forwards half speed
		Wait(.2); 				//    for 2- seconds
	

		myRobot.Drive(0.0, 0.0);

	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		float sensitivity = -.9;
		bool drive_style = true;

		myRobot.SetSafetyEnabled(false);
		myRobot.SetExpiration(5);
		
		DriverStationLCD *ds = DriverStationLCD::GetInstance();

		float val = myRobot.GetExpiration();

		ds->PrintfLine(DriverStationLCD::kUser_Line1, "Expiration Value: %d", val);
		ds->UpdateLCD();

		myRobot.SetInvertedMotor(myRobot.kRearLeftMotor,true);
		myRobot.SetInvertedMotor(myRobot.kRearRightMotor,true);
		
		timer_one_ms.Start();
		
		bool stable_trigger;
		
		static unsigned short RJS_button_input_state[NUM_BUTTONS];
		static bool RJS_stable_button_value[NUM_BUTTONS];
		static unsigned short LJS_button_input_state[NUM_BUTTONS];
		static bool LJS_stable_button_value[NUM_BUTTONS];
		
		while(IsOperatorControl())
		{
			if (timer_one_ms.Get() == .1)
			{
				//right Joystick trigger
				static unsigned short trigger_input_state = 0x0000;
				trigger_input_state = (trigger_input_state << 1) | (right_jstick.GetTrigger()) | (0x1FFF);
				if (trigger_input_state == 0x0FFF)
				{
					stable_trigger = true;
				}
				else
				{
					stable_trigger = false;
				}
				
				//right joystick buttons


				unsigned char button_count;
				for ((button_count = 0); (button_count < NUM_BUTTONS); (button_count++))
				{
					RJS_button_input_state[button_count] = (RJS_button_input_state[button_count] << 1) | (right_jstick.GetRawButton(button_count)) | (0x1FFF);  //this mask might need to change
					if (RJS_button_input_state[button_count] == 0x0FFF)
					{
						RJS_stable_button_value[button_count] = true;
					}
					else
					{
						RJS_stable_button_value[button_count] = false;
					}				
				
				}
				//left joystick buttons

				for ((button_count = 0); (button_count < NUM_BUTTONS); (button_count++))
				{
					LJS_button_input_state[button_count] = (LJS_button_input_state[button_count] << 1) | (left_jstick.GetRawButton(button_count)) | (0x1FFF);  //this mask might need to change
					if (LJS_button_input_state[button_count] == 0x0FFF)
					{
						LJS_stable_button_value[button_count] = true;
					}
					else
					{
						LJS_stable_button_value[button_count] = false;
					}				
				}			
				
				
				timer_one_ms.Reset();
			}
			

		//Determin drive Style
			if (stable_trigger == drive_style)//(right_jstick.GetTrigger() == drive_style)
			{
				drive_style = false;
				myRobot.TankDrive(left_jstick, right_jstick);
				Wait(0.001);// wait for a motor update time
				
			}
			else 
			{
				drive_style = true;
				myRobot.ArcadeDrive(right_jstick);
				Wait(0.001);				// wait for a motor update time
			}

			if (drive_style = true)
			{
#warning no idea if the servo control code will work.

				float camera_yaw_position = camera_yaw_servo.Get();
				float camera_tilt_position = camera_tilt_servo.Get();
				if (left_jstick.GetY() > 0)
				{
					camera_yaw_position += 0.01;
				}
				else if(left_jstick.GetY() < 0)
				{
					camera_yaw_position -= 0.01;
				}
				else
				{
					
				}
	
				if (left_jstick.GetX() > 0)
				{
					camera_tilt_position += 0.01;
				}
				else if(left_jstick.GetX() < 0)
				{
					camera_tilt_position -= 0.01;
				}
				else
				{
					
				}				
				camera_yaw_servo.Set(camera_yaw_position);
				camera_tilt_servo.Set(camera_tilt_position);
				
			}
			else
			{
#warning this should home the servo, it probably will not
				camera_yaw_servo.SetAngle(0);
				camera_tilt_servo.SetAngle(0);
			}

			if(RJS_stable_button_value[2])
			{
				motor_shooter.Set(.5);
			}
			else
			{
				motor_shooter.Disable();
			}



			if(RJS_stable_button_value[8])
			{
				sensitivity-=.005;
				if(sensitivity > 10)
				{
					sensitivity = 10;
				}

				myRobot.SetSensitivity(sensitivity);
				ds->PrintfLine(DriverStationLCD::kUser_Line4, "Sensitivity = %f", sensitivity);
			}

			if(RJS_stable_button_value[9])
			{
				sensitivity+=.005;
				if(sensitivity > 10)
				{
					sensitivity = 10;
				}
				myRobot.SetSensitivity(sensitivity);
				ds->PrintfLine(DriverStationLCD::kUser_Line4, "Sensitivity = %f", sensitivity);
			}
			
			ds->UpdateLCD();
		}
	}

	/**
	 * Runs during test mode
	 */
	void Test() {
		myRobot.SetSafetyEnabled(true);
		while (IsTest())
		{
			myRobot.TankDrive(left_jstick, right_jstick); // drive with arcade style (use right stick)
			Wait(0.005);				// wait for a motor update time
		}

	}
};

START_ROBOT_CLASS(RobotDemo);

