#include "WPILib.h"
#include <PWM.h>
#include "Accelerometer.h"// should be using I2C interface, NOT ANALOG!!!!!!
//#include <Jaguar.h>


class RobotDemo : public SimpleRobot
{
	RobotDrive myRobot; // robot drive system
	Joystick left_jstick; // only joystick
	Joystick right_jstick;
	Jaguar motor_shooter;
	Timer drive_timer;
	Timer timer;
	Timer trigger_timer;
	Gyro gyro;
	Accelerometer accel;
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
		gyro(1),
		accel(2)
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
		myRobot.Drive(0.0, 0.0); 	// stop robot
		Wait(.5);
		myRobot.Drive(.2, .5);
		Wait(.2);

		printf("here 2");
		myRobot.Drive(.2, 0.0); 	// drive forwards half speed
		Wait(.2); 				//    for 2- seconds
		myRobot.Drive(0.0, 0.0); 	// stop robot
		Wait(.5);
		myRobot.Drive(.2, .5);
		Wait(.2);

		myRobot.Drive(0.0, 0.0);

	}

	/**
	 * Runs the motors with arcade steering. 
	 */
	void OperatorControl(void)
	{
		float voltage;
		float sensitivity = .25;
		bool drive_style = true;
		gyro.Reset();
		float angle = gyro.GetAngle();  // current heading (0 = target)
		float acceleration = accel.GetAcceleration();
		myRobot.SetSafetyEnabled(false);

		//char name = 'body';
		//char *sysName;
		//sysName = &name;

		//Subsystem body = Subsystem(sysName);

		DriverStationLCD *ds = DriverStationLCD::GetInstance();
		DriverStation *station = DriverStation::GetInstance();

		//LiveWindow *lw = LiveWindow::GetInstance();
		//lw->AddSensor('body','Accelerometer',accel);
		//lw->AddSensor()
		//myRobot.SetExpiration(5);

		float val = myRobot.GetExpiration();

		ds->PrintfLine(DriverStationLCD::kUser_Line1, "Expiration Value: %d", val);
		ds->PrintfLine(DriverStationLCD::kUser_Line5, "Gyro Value: %d", angle);
		ds->PrintfLine(DriverStationLCD::kUser_Line6, "Acceleration: %d", acceleration);
		ds->UpdateLCD();

		myRobot.SetInvertedMotor(myRobot.kRearLeftMotor,true);
		myRobot.SetInvertedMotor(myRobot.kRearRightMotor,true);
		drive_timer.Start();
		timer.Start();

		bool current_trigger;
		bool previous_trigger;
		bool stable_trigger;

		unsigned int trigger_state = 0;
		//		unsigned int trigger_count = 0;
		const double trigger_bounce_time_S = .1;  //this is in seconds//#fix change to a #define 

		while(IsOperatorControl())
		{
			previous_trigger = current_trigger;
			current_trigger = right_jstick.GetTrigger();

			switch (trigger_state)
			{
			case 0:  //No Button Press 
				//current_trigger = right_jstick.GetTrigger();					
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

			//if (drive_timer.Get() > .1){


			if (stable_trigger == drive_style)//(right_jstick.GetTrigger() == drive_style)
			{
				drive_style=false;
				myRobot.TankDrive(left_jstick, right_jstick);
				Wait(0.001);// wait for a motor update time

			}
			else 
			{
				drive_style=true;
				myRobot.ArcadeDrive(right_jstick);
				Wait(0.001);				// wait for a motor update time
			}

			/*
			 * if (right_jstick.GetTrigger() == drive_style)
				{
					drive_style=false;			
					myRobot.TankDrive(left_jstick, right_jstick);
					Wait(0.001);				// wait for a motor update time
				}
				else 
				{
					drive_style=true;
					myRobot.ArcadeDrive(right_jstick);
					Wait(0.001);				// wait for a motor update time
				}

			 */
			drive_timer.Reset();
			//}

			if(right_jstick.GetRawButton(2))
			{
				motor_shooter.Set(.5);
			}
			else
			{
				motor_shooter.Disable();
			}



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
			}
			//ds->PrintfLine(DriverStationLCD::kUser_Line1, "trigger_state", trigger_state);	
			angle= gyro.GetAngle();
			acceleration= accel.GetAcceleration();
			ds->PrintfLine(DriverStationLCD::kUser_Line5, "Gyro Value: %d", angle);
			ds->PrintfLine(DriverStationLCD::kUser_Line6, "Acceleration: %d", acceleration);
			voltage = station->GetBatteryVoltage();
			if(voltage<10){
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

