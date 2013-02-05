#include "WPILib.h"
#include <PWM.h>


class RobotDemo : public SimpleRobot
{
	RobotDrive myRobot; // robot drive system
	Joystick left_jstick; // only joystick
	Joystick right_jstick;
	Jaguar motor_shooter_front;
	Jaguar motor_shooter_back;
	DoubleSolenoid dub_sol;
	Timer drive_timer;
	Timer timer;
	Timer trigger_timer;
	Timer gyro_timer;
	Encoder encoder;
	ADXL345_I2C accel;
	Timer accel_timer;
	Gyro gyro;
	Servo camera_yaw_servo;
	Servo camera_pitch_servo;
	
	static const float m_sensitivity = .25;
	static const int NUM_JOYSTICK_BUTTONS = 16;
	bool m_rightStickButtonState[(NUM_JOYSTICK_BUTTONS+1)];
	bool m_leftStickButtonState[(NUM_JOYSTICK_BUTTONS+1)];	


public:

	RobotDemo(void):
		myRobot(2,1),	                                    //these must be initialized in the same order
		left_jstick(1),		                                //as they are declared above.
		right_jstick(2),
		motor_shooter_front(3),
		motor_shooter_back(4),
		dub_sol(1,2),                                       //(forwardChannel,reverseChannel)
		drive_timer(),
		timer(),
		trigger_timer(),
		gyro_timer(),
		gyro(2),
		camera_yaw_servo(5),
		camera_pitch_servo(6),
		accel(1,accel.kRange_2G),
		accel_timer(),
		encoder(1024,4096, false, encoder.k4X)
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
		myRobot.Drive(.2, 0.0); 	                         //drive forwards half speed
		Wait(.2); 				                             //for 2- seconds
		myRobot.Drive(0.0, 0.0); 	                         //stop robot
		Wait(.5);
		myRobot.Drive(.2, .5);
		Wait(.2);

		printf("here 2");
		myRobot.Drive(.2, 0.0); 	                         //drive forwards half speed
		Wait(.2); 				                             //for 2- seconds
		myRobot.Drive(0.0, 0.0); 	                         //stop robot
		Wait(.5);
		myRobot.Drive(.2, .5);
		Wait(.2);

		myRobot.Drive(0.0, 0.0);

	}

	/**
	 * Runs the motors with arcade or tank steering (the trigger toggles). 
	 */
	void OperatorControl(void)
	{
		encoder.Start();
		encoder.Reset();
		float voltage;
		float sensitivity = .25;
		bool drive_style = true;
		gyro.Reset();
		double angle = 0.0;
		float accelerationX = 0.0;
		float accelerationY = 0.0;
		float accelerationZ = 0.0;
		myRobot.SetSafetyEnabled(false);
		
		DriverStationLCD *ds = DriverStationLCD::GetInstance();
		DriverStation *station = DriverStation::GetInstance();
		
		Dashboard dashboard(station->GetUserStatusDataSem());
		
		ds->UpdateLCD();

		myRobot.SetInvertedMotor(myRobot.kRearLeftMotor,true);
		myRobot.SetInvertedMotor(myRobot.kRearRightMotor,true);
		drive_timer.Start();
		timer.Start();
		gyro_timer.Start();
		accel_timer.Start();

		bool current_trigger;
		bool previous_trigger;
		bool stable_trigger;

		unsigned int trigger_state = 0;
		const double trigger_bounce_time_S = .1;                       //this is in seconds//#fix change to a #define 

		dashboard.AddFloat(voltage);
		dashboard.AddFloat(sensitivity);
		dashboard.AddDouble(angle);
		dashboard.AddDouble(encoder.GetRate());	
		dashboard.AddFloat(accelerationX);
		dashboard.AddFloat(accelerationY);
		dashboard.AddFloat(accelerationZ);
		dashboard.Finalize();
		
		while(IsOperatorControl())
		{
			previous_trigger = current_trigger;
			current_trigger = right_jstick.GetTrigger();

			switch (trigger_state)
			{
			case 0: //No Button Pressed
				if (current_trigger)
				{
					trigger_timer.Reset();
					trigger_state = 1;
				}
				else
				{
					trigger_state = 0;
				}
				break;
			case 1:
				trigger_timer.Start();
				trigger_state = 2;
				break;
			case 2:
				if (trigger_timer.Get() > trigger_bounce_time_S)
				{
					trigger_timer.Stop();
					trigger_timer.Reset();
					trigger_state = 3;
				}
				else
				{
					trigger_state = 2;
				}

				break;
			case 3:

				stable_trigger = current_trigger;
				if (previous_trigger != current_trigger)
				{
					trigger_state = 0;
				}
				else
				{

					previous_trigger = current_trigger;
				}
				break;
			default:
				trigger_state = 0;
				break;
			}
			ds->PrintfLine(DriverStationLCD::kUser_Line1, "trigger_state", trigger_state);

			if (stable_trigger == drive_style)                
			{
				drive_style=false;
				myRobot.TankDrive(left_jstick, right_jstick);
				Wait(0.001);                                   // wait for a motor update time
				camera_yaw_servo.SetAngle(0);
				camera_pitch_servo.SetAngle(0);
			}
			else 
			{
				drive_style=true;
				myRobot.ArcadeDrive(right_jstick);
				Wait(0.001);				                  // wait for a motor update time
			}

			if(right_jstick.GetRawButton(2))
			{
				motor_shooter_front.Set(.6);
				motor_shooter_back.Set(.6);
			}
			else
			{
				motor_shooter_front.Disable();
				motor_shooter_back.Disable();
			}

			drive_timer.Reset();
			if (timer.Get() > .005)
			{
				if(right_jstick.GetRawButton(8))
				{
					sensitivity-=.005;
					if(sensitivity > 10)
					{
						sensitivity = 10;
					}

					myRobot.SetSensitivity(sensitivity);
					ds->PrintfLine(DriverStationLCD::kUser_Line4, "Sensitivity = %f", sensitivity);
				}

				if(right_jstick.GetRawButton(9))
				{
					sensitivity+=.005;
					if(sensitivity > 10)
					{
						sensitivity = 10;
					}
					myRobot.SetSensitivity(sensitivity);
					ds->PrintfLine(DriverStationLCD::kUser_Line4, "Sensitivity = %f", sensitivity);
				}
				timer.Reset();

				//Camera movement control rotate
				//4 left, 5 right
				if (left_jstick.GetRawButton(4))
				{
					camera_yaw_servo.Set(camera_yaw_servo.Get()-0.01);
				}
				else if (left_jstick.GetRawButton(5))
				{
					camera_yaw_servo.Set(camera_yaw_servo.Get()+0.01);
				}

				//Camera movement control up/down
				//2 down, 3 up
				if (left_jstick.GetRawButton(3))
				{
					camera_pitch_servo.Set(camera_pitch_servo.Get()-0.01);
				}
				else if (left_jstick.GetRawButton(2))
				{
					camera_pitch_servo.Set(camera_pitch_servo.Get()+0.01);
				}
				else 
				{
					//	camera_yaw_servo.SetOffline();
				}

				//DoubleSolenoid control
				//6 forward, 7 reverse, 10 don't move
				if(left_jstick.GetRawButton(6))
				{
					dub_sol.Set(dub_sol.kForward);
				}
				else if(left_jstick.GetRawButton(7))
				{
					dub_sol.Set(dub_sol.kReverse);
				}
				else if(left_jstick.GetRawButton(10))
				{
					dub_sol.Set(dub_sol.kOff);
				}
			}

			//Prints gyro
			//Refreshes gyro twice every second
			if(gyro_timer.Get() > .5)
			{
				angle = gyro.GetAngle();
				ds->PrintfLine(DriverStationLCD::kUser_Line5, "Gyro Value: %f", angle);

				gyro_timer.Reset();
			}
			
			//Prints accelerometer
			//Refreshes accelerometer twice every second
			if(accel_timer.Get() > .5)
			{
				accelerationX = accel.GetAcceleration(accel.kAxis_X);
				accelerationY = accel.GetAcceleration(accel.kAxis_Y);
				accelerationZ = accel.GetAcceleration(accel.kAxis_Z);
				ds->Printf(DriverStationLCD::kUser_Line4, 1, "X Axis: %f", accelerationX);//dont keep kUser_line4
				ds->Printf(DriverStationLCD::kUser_Line5, 1, "Y Axis: %f",accelerationY);//dont keep kUser_Line5		
				ds->Printf(DriverStationLCD::kUser_Line6, 1, "Z Axis: %f",accelerationZ);
									
				accel_timer.Reset();
			}

			//Prints if the battery is low or fine
			//if the voltage is below 10 the battery is low
			voltage = station->GetBatteryVoltage();
			if(voltage<11){
				ds->PrintfLine(DriverStationLCD::kUser_Line3, "Battery is low..");
			}
			else
			{
				ds->PrintfLine(DriverStationLCD::kUser_Line3, "Battery is fine");
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

