#include "WPILib.h"
#include <PWM.h>

<<<<<<< HEAD
#define port1 1
#define port2 2
#define motor1 2
#define motor2 1
#define motor3 3
#define motor4 4
#define hopper 8
#define solChan1 1
#define solChan2 2


=======
>>>>>>> parent of a506b41... 2-16-2013 Morning
class RobotDemo: public SimpleRobot {
	RobotDrive myRobot; // robot drive system
	Joystick left_jstick; // only joystick
	Joystick right_jstick;
	Jaguar motor_shooter_front; //adding extra motors for the shooter
	Jaguar motor_shooter_back;
	Jaguar hopper_loader_arm;
	DoubleSolenoid dub_sol;
	Timer drive_timer; //adding timers to control how fast the buttons register
	Timer timer;
	Timer trigger_timer;
<<<<<<< HEAD
	Compressor compressor;
=======
	Timer gyro_timer;
	Encoder encoder;
	ADXL345_I2C accel;
	Timer accel_timer;
	Gyro gyro;
	Servo camera_yaw_servo;
	Servo camera_pitch_servo;
	Servo hopper_yaw_servo;
>>>>>>> parent of a506b41... 2-16-2013 Morning

	static const float m_sensitivity = .25;
	static const int NUM_JOYSTICK_BUTTONS = 16;
	bool m_rightStickButtonState[(NUM_JOYSTICK_BUTTONS + 1)];
	bool m_leftStickButtonState[(NUM_JOYSTICK_BUTTONS + 1)];

public:

	float SensitivityCheck(Timer timer);
	// returns the sensitivity of the robot and changes it to fit that sensitivity
	void ArcadeMovement(float move, float rotate);
	void TankMovement(float left, float right);
	
	
	RobotDemo(void) :
				myRobot(2, 1), //these must be initialized in the same order
				left_jstick(1), //as they are declared above.
				right_jstick(2),
				motor_shooter_front(3),
				motor_shooter_back(4),
				hopper_loader_arm(8),
				dub_sol(1, 2), //(forwardChannel,reverseChannel)
				drive_timer(), 
				timer(), 
				trigger_timer(), 
<<<<<<< HEAD
				compressor(8,7)
=======
				gyro_timer(),
				encoder(1024, 4096, false, encoder.k4X),
				accel(1, accel.kRange_2G), 
				accel_timer(), 
				gyro(2),
				camera_yaw_servo(5), 
				camera_pitch_servo(6), 
				hopper_yaw_servo(7) 
>>>>>>> parent of a506b41... 2-16-2013 Morning
	{
		myRobot.SetExpiration(0.1); //Leave in, no idea what it does
	}
	
	/*
	 * Runs the shooter at 80% until taken off of autonomous
	 * Must manually aim
	 */
	void Autonomous(void) 
	{
		dub_sol.Set(dub_sol.kForward);// Reverses Solenoid
		dub_sol.Set(dub_sol.kOff);    // Turns Solenoid Off
		compressor.Start();//Starts Compressor
		
		motor_shooter_front.Set(.8);//Runs Front Motor
		motor_shooter_back.Set(.8);//Runs Back Motor
		hopper_loader_arm.Set(1);//Runs Loader Arm

	}

	/**
	 * Runs the motors with arcade or tank steering (the trigger toggles). 
	 */
	void OperatorControl(void) {
		float voltage;
		float sensitivity = .25;
		bool drive_style = true;
		double angle = 0.0;
		float accelerationX = 0.0;
		float accelerationY = 0.0;
		float accelerationZ = 0.0;
		myRobot.SetSafetyEnabled(false);

		DriverStationLCD *ds = DriverStationLCD::GetInstance();
		DriverStation *station = DriverStation::GetInstance();

		Dashboard dashboard(station->GetUserStatusDataSem());

		myRobot.SetInvertedMotor(myRobot.kRearLeftMotor, true);
		myRobot.SetInvertedMotor(myRobot.kRearRightMotor, true);
		timer.Start();
<<<<<<< HEAD
		drive_timer.Start();
		compressor.Start();
		
=======
		gyro_timer.Start();
		accel_timer.Start();

>>>>>>> parent of a506b41... 2-16-2013 Morning
		bool current_trigger;
		bool previous_trigger;
		bool stable_trigger;

		unsigned int trigger_state = 0;
		const double trigger_bounce_time_S = .1; //this is in seconds//#fix change to a #define 

		dashboard.AddFloat(voltage);
		dashboard.AddFloat(sensitivity);
		dashboard.AddDouble(angle);
		dashboard.AddDouble(accelerationX);
		dashboard.AddDouble(accelerationY);
		dashboard.AddDouble(accelerationZ);
		dashboard.Finalize();

		while (IsOperatorControl()) {

  		previous_trigger = current_trigger;
			current_trigger = right_jstick.GetTrigger();

			//used to toggle the drive types
			switch (trigger_state) {
			case 0: //No Button Pressed
				if (current_trigger) {
					trigger_timer.Reset();
					trigger_state = 1;
				} else {
					trigger_state = 0;
				}
				break;
			case 1:
				trigger_timer.Reset();
				trigger_timer.Start();
				trigger_state = 2;
				break;
			case 2:
				if (trigger_timer.Get() > trigger_bounce_time_S) {
					trigger_timer.Stop();
					trigger_timer.Reset();
					trigger_state = 3;
				} else {
					trigger_state = 2;
				}

				break;
			case 3:

				stable_trigger = current_trigger;
				if (previous_trigger != current_trigger) {
					trigger_state = 0;
				} else {

					previous_trigger = current_trigger;
				}
				break;
			default:
				trigger_state = 0;
				break;
			}
<<<<<<< HEAD
			ds->PrintfLine(DriverStationLCD::kUser_Line1, "trigger_state %d",
					stable_trigger);
=======
			ds->PrintfLine(DriverStationLCD::kUser_Line1, "trigger_state",
					trigger_state);
>>>>>>> parent of a506b41... 2-16-2013 Morning

//			stable_trigger = right_jstick.GetTrigger();
			
			if (stable_trigger == drive_style) 
			{
				drive_style = false;
				myRobot.TankDrive(left_jstick, right_jstick);
			}
			else
			{
				drive_style = true;
				myRobot.ArcadeDrive(right_jstick);
				Wait(0.001); // wait for a motor update time
			}
			
			drive_timer.Reset();
			
			//runs the shooter motors
			if (right_jstick.GetRawButton(2)) {
				motor_shooter_front.Set(.8);
				motor_shooter_back.Set(.8);
			} 
			else if (left_jstick.GetRawButton(2)) {
				motor_shooter_front.Set(.3);
				motor_shooter_back.Set(.3);
			}			
			else {
				motor_shooter_front.Disable();
				motor_shooter_back.Disable();
			}
			drive_timer.Reset();

			//sets the sensitivity
			sensitivity = SensitivityCheck();
			ds->PrintfLine(DriverStationLCD::kUser_Line2, "Sensitivity = %f",
					sensitivity);
			
			//hopper server moving .25 of a turn and back
			if (right_jstick.GetRawButton(3)
				|| left_jstick.GetRawButton(3)
				) 
			{
				hopper_loader_arm.Set(1);				
			} else// if (hopper_yaw_servo.Get() >= -0.25 && hopper_yaw_servo.Get() <= 0.25) {
			{
				hopper_loader_arm.Disable();		
			}

			
			//DoubleSolenoid control
			//6 forward, 7 reverse, 10 don't move
			if (left_jstick.GetRawButton(6)) {
				dub_sol.Set(dub_sol.kForward);
			} else if (left_jstick.GetRawButton(7)) {
				dub_sol.Set(dub_sol.kReverse);
			} else if (left_jstick.GetRawButton(10)) {
				dub_sol.Set(dub_sol.kOff);
			}
<<<<<<< HEAD
			
=======

			//Prints gyro
			//Refreshes gyro twice every second
			if (gyro_timer.Get() > .5) {
				angle = gyro.GetAngle();
				ds->PrintfLine(DriverStationLCD::kUser_Line2, "Gyro Value: %f",
						angle);

				gyro_timer.Reset();
			}

			//Prints accelerometer
			//Refreshes accelerometer twice every second
			if (accel_timer.Get() > .5) {
				accelerationX = accel.GetAcceleration(accel.kAxis_X);
				accelerationY = accel.GetAcceleration(accel.kAxis_Y);
				accelerationZ = accel.GetAcceleration(accel.kAxis_Z);
				ds->Printf(DriverStationLCD::kUser_Line4, 1, "X Axis: %f",
						accelerationX);//dont keep kUser_line4
				ds->Printf(DriverStationLCD::kUser_Line5, 1, "Y Axis: %f",
						accelerationY);//dont keep kUser_Line5		
				ds->Printf(DriverStationLCD::kUser_Line6, 1, "Z Axis: %f",
						accelerationZ);

				accel_timer.Reset();
			}

>>>>>>> parent of a506b41... 2-16-2013 Morning
			//Prints if the battery is low or fine
			//if the voltage is below 10 the battery is low
			voltage = station->GetBatteryVoltage();
			if (voltage < 11) {
				ds->PrintfLine(DriverStationLCD::kUser_Line3,
						"Battery is low..");
			} else {
				ds->PrintfLine(DriverStationLCD::kUser_Line3, "Battery is fine");
			}

			ds->UpdateLCD();
		}
	}

	//returns the sensitivity of the robot, and sets it 
	float SensitivityCheck(void)//Timer timer)
	{
		float sensitivity = .25;
		if (timer.Get() > .005) {
			if (right_jstick.GetRawButton(8)) {
				sensitivity -= .005;
				if (sensitivity > 10) {
					sensitivity = 10;
				}

				myRobot.SetSensitivity(sensitivity);
			}

			if (right_jstick.GetRawButton(9)) {
				sensitivity += .005;
				if (sensitivity > 10) {
					sensitivity = 10;
				}
				myRobot.SetSensitivity(sensitivity);
			}
			timer.Reset();
		}
		return sensitivity;
	}

	/**
	 * Runs during test mode
	 */
	void Test() {
		myRobot.SetSafetyEnabled(true);
		while (IsTest()) {
			myRobot.TankDrive(left_jstick, right_jstick); // drive with arcade style (use right stick)
			Wait(0.005); // wait for a motor update time
		}
	}
	
};

START_ROBOT_CLASS(RobotDemo)
;
