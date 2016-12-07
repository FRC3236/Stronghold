#include <iostream>
#include <cmath>
#include <unistd.h>
#include "WPILib.h"
#include "Timer.h"
//#include "LHSVision.cpp"

const double	SMOOTH_DRIVE_P_GAIN		=	0.5;
const double	DRIVE_DEADZONE			=	0.05;
const double	DRIVE_X_TOLERANCE		=	0.05;
const double	ANGLE_TOLERANCE			=	0.1;

const double	DRIVE_P_GAINT			=	1;
const double	DRIVE_I_GAINT			=	0.05;
const double	DRIVE_D_GAINT			=	2.0;
const double	DRIVE_K					=	0.01;

const double	TURN_P_GAIN				=	1.0;
const double	TURN_I_GAIN				=	0.05;
const double	TURN_D_GAIN				=	2.0;
const double	TURN_K					=	0.001;

const double	TURN_P_MAX				=	45;
const double	TURN_I_MAX				=	360;
const double	TURN_D_MAX				=	360;

const double	PITCH_P_GAIN			=	1;
const double	PITCH_I_GAIN			=	0.5;
const double	PITCH_D_GAIN			=	2;
const double	PITCH_K					=	0.001;

double	ACCEL_CALIBRATION				=	0;

//	Movement Tracking
double	WHEEL_DIAMETER					=	0.079121;
int		ENCODER_SLOTS					=	96;

double	distLeft						=	0;
double	distRight						=	0;

double	speedLeft						=	0;
double	speedRight						=	0;

bool	driveStraight					=	false;

double	turnP							=	0;
double	turnI							=	0;
double	turnD							=	0;
double	turnInterval					=	0;

double	pitch							=	0;
double	pitchSpeed						=	0;
double	pitchP							=	0;
double	pitchI							=	0;
double	pitchD							=	0;

double	drivePower						=	0;
double	drivePowerLeft					=	0;
double	drivePowerRight					=	0;
double	turnPower						=	0;
double	targetAngle						=	0;

int		autoDriveState					=	0;
double	acceleration					=	0;
double	speed							=	0;
double	distance						=	0;
bool	tracking						=	false;

int		autoState						=	0;

double times;
double aTimer;

double accelX;
double accelY;
double accelZ;

class Robot: public IterativeRobot
{
	LiveWindow	*lw = LiveWindow::GetInstance();

	SendableChooser	*autoChooser;
	SendableChooser	*teleChooser;
	const std::string	autoNameDefault = "Default";

	const std::string	autoNameRamparts = "Ramparts";
	const std::string	autoNameLowbar = "Lowbar";
	const std::string	autoNameRoughTerrain = "Rough Terrain";
	const std::string	autoNameSallyGate = "Sally Gate (aka the 'sadly gate')";
	const std::string	autoNameTheFrenchOne = "CDF";
	const std::string	autoNameDrawBridge = "Generic";
	const std::string	autoNamePortcullis = "Portcullis";
	const std::string	autoNameRockWall = "Rock Wall";

	std::string autoSelected;

	const std::string teleNameDefault = "BothSticks";
	const std::string teleNameSingle = "****SingleStick";

	std::string teleSelected;

	Timer	timer;
	Timer 	autoTimer;
	RobotDrive Robotc;
	Joystick		driverStick;
	JoystickButton	driverTrigger;
	JoystickButton	driverThumb;
	JoystickButton	driverB3;
	JoystickButton	driverB4;
	JoystickButton	driverB5;
	JoystickButton	driverB6;
	JoystickButton	driverB7;
	JoystickButton	driverB8;
	JoystickButton	driverB9;
	JoystickButton	driverB10;
	JoystickButton	driverB11;
	JoystickButton	driverB12;

	Joystick		operatorStick;
	JoystickButton	operatorTrigger;
	JoystickButton	operatorThumb;
	JoystickButton	operatorB3;
	JoystickButton	operatorB4;
	JoystickButton	operatorB5;
	JoystickButton	operatorB6;
	JoystickButton	operatorB7;
	JoystickButton	operatorB8;
	JoystickButton	operatorB9;
	JoystickButton	operatorB10;
	JoystickButton	operatorB11;
	JoystickButton	operatorB12;

	CANTalon		driveLeft1;
	CANTalon		driveLeft2;
	CANTalon		driveLeft3;
	CANTalon		driveRight1;
	CANTalon 		driveRight2;
	CANTalon		driveRight3;
	CANTalon		pitch;

	Talon			roller;
	Talon			intake1;
	Talon			intake2;

	AnalogGyro		gyro;
	ADXL345_I2C		accel;

	Encoder			encoderArm;
	Encoder			encoderLeft;
	Encoder			encoderRight;

	DigitalInput	intakeClosed;
	DigitalInput	intakeOpen;
	DigitalInput	pitchDown;
	DigitalInput	armDown;

	Relay	*AR = new Relay(0);
	Relay	*AL = new Relay(1);

public:
	Robot():
		Robotc(1, 2),

		driverStick		( 0 ),
		driverTrigger	( &driverStick , 1 ),
		driverThumb		( &driverStick , 2 ),
		driverB3		( &driverStick , 3 ),
		driverB4		( &driverStick , 4 ),
		driverB5		( &driverStick , 5 ),
		driverB6		( &driverStick , 6 ),
		driverB7		( &driverStick , 7 ),
		driverB8		( &driverStick , 8 ),
		driverB9		( &driverStick , 9 ),
		driverB10		( &driverStick , 10 ),
		driverB11		( &driverStick , 11 ),
		driverB12		( &driverStick , 12 ),

		operatorStick	( 1 ),
		operatorTrigger	( &operatorStick , 1 ),
		operatorThumb	( &operatorStick , 2 ),
		operatorB3		( &operatorStick , 3 ),
		operatorB4		( &operatorStick , 4 ),
		operatorB5		( &operatorStick , 5 ),
		operatorB6		( &operatorStick , 6 ),
		operatorB7		( &operatorStick , 7 ),
		operatorB8		( &operatorStick , 8 ),
		operatorB9		( &operatorStick , 9 ),
		operatorB10		( &operatorStick , 10 ),
		operatorB11		( &operatorStick , 11 ),
		operatorB12		( &operatorStick , 12 ),

		//	SRX TALONS
		driveLeft1		( 1 ),
		driveLeft2		( 4 ),
		driveLeft3		( 5 ),
		driveRight1		( 2 ),
		driveRight2		( 3 ),
		driveRight3		( 6 ),
		pitch			( 7 ),

		//	OLD TALONS
		roller			( 2	),
		intake1			( 0 ),
		intake2			( 1 ),

		gyro			( 0 ),
		autoChooser		(  ),
		teleChooser		(  ),
		accel			( I2C::Port::kOnboard ),

		encoderArm		( 4 , 5 ),
		encoderLeft		( 6 , 7 ),
		encoderRight	( 8 , 9 ),

		intakeClosed	( 0 ),
		intakeOpen		( 1 ),
		pitchDown		( 2 ),
		armDown			( 3	)
	{}

	void RobotInit()
	{
		autoChooser = new SendableChooser();
		autoChooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		autoChooser->AddObject(autoNameRamparts, (void*)&autoNameRamparts);
		autoChooser->AddObject(autoNameLowbar, (void*)&autoNameLowbar);
		autoChooser->AddObject(autoNameRoughTerrain, (void*)&autoNameRoughTerrain);
		autoChooser->AddObject(autoNameSallyGate, (void*)&autoNameSallyGate);
		autoChooser->AddObject(autoNameTheFrenchOne, (void*)&autoNameTheFrenchOne);
		autoChooser->AddObject(autoNameDrawBridge, (void*)&autoNameDrawBridge);
		autoChooser->AddObject(autoNamePortcullis, (void*)&autoNamePortcullis);
		autoChooser->AddObject(autoNameRockWall, (void*)&autoNameRockWall);

		SmartDashboard::PutData("Auto Modes", autoChooser);

		teleChooser = new SendableChooser();
		teleChooser->AddDefault(teleNameDefault, (void*)&teleNameDefault);
		teleChooser->AddObject(teleNameSingle, (void*)&teleNameSingle);

		SmartDashboard::PutData("Tele Modes", teleChooser);

		AR->Set(Relay::Value::kOff);
		AL->Set(Relay::Value::kOff);

//		mLHSVision = new LHSVision(Robotc, driverStick);

//		if (fork() == 0)						//Creating process for grip and testing if it fails
//		{
//			if (execv(JAVA, GRIP_ARGS) == -1)
//			{
//				perror("Error running GRIP");
//		    }
//		}

		timer.Start();

//		for ( int n = 0 ; n < 256 ; n++ )
//		{
//			Wait(0.1);
//			ACCEL_CALIBRATION	+=	accel.GetY();
//		}
//		ACCEL_CALIBRATION	/=	256;

		gyro.Calibrate();

		encoderLeft.SetDistancePerPulse(	(double)	M_PI	*	WHEEL_DIAMETER	/	ENCODER_SLOTS	);
		encoderRight.SetDistancePerPulse(	(double)	M_PI	*	WHEEL_DIAMETER	/	ENCODER_SLOTS	);

		CameraServer::GetInstance()->SetQuality(50);
		CameraServer::GetInstance()->StartAutomaticCapture("cam0");
//		CameraServer::GetInstance()->StartAutomaticCapture("cam2");
//		Cam0.SetExposureAuto();
//		Cam1.SetExposureAuto();

	}


	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the GetString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the if-else structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
	void AutonomousInit()
	{
		AR->Set(Relay::Value::kOn);
		AL->Set(Relay::Value::kOff);
		autoTimer.Reset();
		autoTimer.Start();

		autoSelected = *((std::string*)autoChooser->GetSelected());
		std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if(autoSelected == autoNameDefault)
		{
			KillAll();
		}
		else if(autoSelected == autoNameRamparts)
		{
			KillAll();
		}
		else if(autoSelected == autoNameLowbar)
		{
			KillAll();
		}
		else if(autoSelected == autoNameRoughTerrain)
		{
			KillAll();
		}
		else if(autoSelected == autoNameSallyGate)
		{
			KillAll();
		}
		else if(autoSelected == autoNameTheFrenchOne)
		{
			KillAll();
		}
		else if(autoSelected == autoNameDrawBridge)
		{
			KillAll();
		}
		else if(autoSelected == autoNamePortcullis)
		{
			KillAll();
		}
		else if(autoSelected == autoNameRockWall)
		{
			KillAll();
		}
		else
		{
			KillAll();
		}
		autoTimer.Reset();
		autoTimer.Start();
	}

	void AutonomousPeriodic()
	{

//		std::cout << "times = " << aTimer << std::endl;

		aTimer = autoTimer.Get();

		if(autoSelected == autoNameDefault)
		{
			KillAll();
		}
		else if(autoSelected == autoNameRamparts)
		{
			if (	aTimer	>	0	&&	aTimer	<	10		)	Drive(	-0.5	,	-0.5	);
			else												KillAll();
		}
		else if(autoSelected == autoNameLowbar)  //Hardwire for lowbar
		{
			if	(	aTimer	>	0	&&	aTimer	<	2.4		)	pitch.Set(	-0.8  );
			if	(	aTimer	>	3.5	&&	aTimer	<	10.5		)	Drive(	-0.5	,	-0.5	);
			else												KillAll();
		}
		else if(autoSelected == autoNameRoughTerrain)
		{
			if	(	aTimer	>	0	&&	aTimer	<	10		)	Drive(	-0.5	,	-0.5	);
			else												KillAll();
		}
		else if(autoSelected == autoNameSallyGate)
		{
			if (	aTimer	>	0	&&	aTimer	<	6		)	Drive(	-0.5	,	-0.5	);
			else												KillAll();
		}
		else if(autoSelected == autoNameTheFrenchOne)
		{
			KillAll();
		}
		else if(autoSelected == autoNameDrawBridge)
		{
			if (	aTimer	>	0	&&	aTimer	<	4		)	Drive(	-0.75	,	-0.75	);
			else												KillAll();
		}
		else if(autoSelected == autoNamePortcullis)	// portcullis
		{
			if	(	aTimer	>	0	&&	aTimer	<	3.4		)	pitch.Set(	-0.5	);
			if	(	aTimer	>	3.4	&&	aTimer	<	9		)	Drive(	-0.75	,	-0.75	);
			else												KillAll();
		}
		else if(autoSelected == autoNameRockWall)
		{
			if	(	aTimer	>	0	&&	aTimer	<	2		)	pitch.Set(	-0.5	);
			if	(	aTimer	>	4	&&	aTimer	<	9		)	Drive(	-0.75	,	-0.75	);
			else												KillAll();
		}
		else
		{
				//Default Auto goes here
//			std::cout<<"/n Time =";
//			std::cout<<timer.Get();
//
//			if (times <= 5)
//			{
//				driveLeft.Set(0.2);
//				driveRight.Set(-0.2);
//			}
//			else if ((times <= 10) && (times > 5))
//			{
//				driveLeft.Set(-0.2);
//				driveRight.Set(0.2);
//			}
//			else
//			{
//				timer.Reset();
//				timer.Start();
//			}


//			std::cout<<"\n cal = ";
//			std::cout<<ACCEL_CALIBRATION;
//			std::cout<<"\t accel = ";
//			std::cout<<accel.GetY();
//			std::cout<<"\t speed = ";
//			std::cout<<speed;
//			std::cout<<"\t dist = ";
//			std::cout<<distance;

			Update();	// must be called at the end of the periodic loop
		}

//		TankDrive(gyro.GetAngle()/90,0);

		auto grip = NetworkTable::GetTable("grip");

		        /* Get published values from GRIP using NetworkTables */
		auto areas = grip->GetNumberArray("targets/area", llvm::ArrayRef<double>());

		for (auto area : areas)
		{
			std::cout << "Got contour with area=" << area << std::endl;
		}

//		Piston->Set(DoubleSolenoid::Value::kForward);
//		Wait(3);
//		Piston->Set(DoubleSolenoid::Value::kReverse);
//		Wait(3);
//		std::cout << "Piston ";
//
//		std::cout<< "\nangle = ";
//		std::cout<< gyro.GetAngle();
//		std::cout<< "\trate = ";
//		std::cout<< gyro.GetRate();

		Update();	// must be called at the end of the periodic loop
	}

	void TeleopInit()
	{
		AL->Set(Relay::Value::kOn);
		AR->Set(Relay::Value::kOff);
//		gyro.Calibrate();

		teleSelected = *((std::string*)teleChooser->GetSelected());
		std::string TeleSelected = SmartDashboard::GetString("Tele Selector", teleNameDefault);
		std::cout << "Tele selected: " << teleSelected << std::endl;
		KillAll();
	}

	void TeleopPeriodic()
	{
		if (teleSelected == teleNameSingle)
		{
			//	DRIVE
			if (	driverB12.Get()	)										KillDrive();
			else															TankDrive(	driverStick.GetRawAxis(0)	,	driverStick.GetRawAxis(1)	);
		}
		else
		{
			double	operatorThrottle	=	1	-	operatorStick.GetRawAxis	(	3	);
			double	driverThrottle		=	1	-	driverStick.GetRawAxis		(	3	);

			//	PITCH
			pitch.Set(	operatorStick.GetRawAxis(1)	);

			//	DRIVE
			if		(	driverB12.Get()	)									KillDrive();
			else															TankDrive(	driverStick.GetRawAxis(0)	,	driverStick.GetRawAxis(1)	);

			//	INTAKE
			if		(	operatorThumb.Get()		)							SetIntake(	-operatorThrottle	);
			else if (	operatorTrigger.Get()	)							SetIntake(	operatorThrottle	);
			else															SetIntake(	0	);

			//	ROLLER
//			if (	operatorStick.GetPOV()	>	-1	)	roller.Set(	cos(	operatorStick.GetPOV()	)	);
//			if		(	operatorB5.Get()	||	driverB6.Get()	)			roller.Set(	1	);
//			else if (	operatorB3.Get()	||	driverB4.Get()	)			roller.Set(	-1	);
//			else															roller.Set(	0	);

		}
		Update();	// must be called at the end of the periodic loop
	}

	void TestPeriodic()
	{
		lw->Run();
	}

	/* Kill Functions */

	/* Unconditionally stop all motors and reset control variables.
	 */
	void	KillAll ()
	{
		driveLeft1.Set	( 0 );
		driveLeft2.Set	( 0 );
		driveLeft3.Set	( 0 );
		driveRight1.Set	( 0 );
		driveRight2.Set	( 0 );
		driveRight3.Set	( 0 );
		drivePowerLeft	=	0;
		drivePowerRight	=	0;
		drivePower		=	0;
		turnPower		=	0;
		TurnPIDReset();

		pitch.Set		( 0 );
	}

	/* Unconditionally stop all drive motors and reset drive control variables.
	 */
	void	KillDrive ()
	{
		driveLeft1.Set	( 0 );
		driveLeft2.Set	( 0 );
		driveLeft3.Set	( 0 );
		driveRight1.Set	( 0 );
		driveRight2.Set	( 0 );
		driveRight3.Set	( 0 );
		drivePowerLeft	=	0;
		drivePowerRight	=	0;
		drivePower		=	0;
		turnPower		=	0;
		TurnPIDReset();

		turnPower		=	0;
		TurnPIDReset();
	}

	/* DRIVE FUNCTIONS */

	void	Drive	( double _left , double _right )
	{
		// set left motors
		driveLeft1.Set	( -_left );
		driveLeft2.Set	( -_left );
		driveLeft3.Set	( -_left );
		// set right motors
		driveRight1.Set	( _right );
		driveRight2.Set	( _right );
		driveRight3.Set	( _right );
	}

	void	SmoothDrive	( double _left , double _right )
	{
		if( fabs( _left ) <= DRIVE_DEADZONE && fabs( _right ) <= DRIVE_DEADZONE )
		{
			drivePowerLeft	= 0;
			drivePowerRight	= 0;
		}
		else
		{
			drivePowerLeft	+= SMOOTH_DRIVE_P_GAIN * (	_left	- 	drivePowerLeft	);
			drivePowerRight	+= SMOOTH_DRIVE_P_GAIN * (	_right	- 	drivePowerRight	);
		}
		Drive( drivePowerLeft , drivePowerRight );
	}

	void	TankDrive	( double _x , double _y )
	{
		Drive( _y - _x , _y + _x );
	}

	void	SmoothTankDrive	( double _x , double _y )
	{
		if( fabs( _x ) <= DRIVE_DEADZONE )
		{
			drivePower		=	SMOOTH_DRIVE_P_GAIN * (	_y	- 	drivePower	);
			drivePowerLeft	=	drivePower;
			drivePowerRight	=	drivePower;
		}
		else
		{
			double	_left	=	_y - _x;
			double	_right	=	_y + _x;
			drivePowerLeft	+=	SMOOTH_DRIVE_P_GAIN * (	_left	- 	drivePowerLeft	);
			drivePowerRight	+=	SMOOTH_DRIVE_P_GAIN * (	_right	- 	drivePowerRight	);
			drivePower		=	fmin( drivePowerLeft , drivePowerRight );
		}
		Drive( drivePowerLeft , drivePowerRight );
	}

	void	KeepAngle	( double _targetAngle , double _drive )
	{

		std::cout<<"angle	=	"<<GetAngle()<<std::endl;

		// calculate angle deviation
		double _currentAngleDeviation = AngularDifference( _targetAngle , GetAngle() );

		// calculate PID
		turnP	=	_currentAngleDeviation;
		turnI	+=	_currentAngleDeviation;
		turnD	=	-gyro.GetRate();

		if (	fabs( _currentAngleDeviation )	<=	ANGLE_TOLERANCE	)
		{
			turnI	=	0;
		}

//		std::cout<<"turnPower	=	"<<turnPower<<std::endl;
		// calculate turn power
		turnPower	=	TURN_K * ( TURN_P_GAIN * turnP + TURN_I_GAIN * turnI + TURN_D_GAIN * turnD );

		// limit turnPower to [-1,+1]
		if		( turnPower > +1 )	turnPower	=	+1;
		else if	( turnPower < -1 )	turnPower	=	-1;

		// calculate drive power
		drivePower	+=	SMOOTH_DRIVE_P_GAIN * (	_drive	- 	drivePower	);

		// drive the robot
		TankDrive( turnPower , drivePower );
	}

	void	SpecialTankDrive	( double _x , double _y )
	{

	}

	/* GYRO FUNCTIONS */

	double	ModAngle	( double angle )
	{
		angle = angle - 360 * floorf( ( angle - 180 ) / 360 ) - 360;
		if ( angle == -180 ) angle = 180;
		return angle;
	}

	double	GetAngle	()
	{
		return ModAngle( gyro.GetAngle() );
	}

	double	AngularDifference	( double left , double right )
	{
//		std::cout<<"\n";
//		std::cout<<left;
//		std::cout<<"-";
//		std::cout<<right;
//		std::cout<<"=";
//		std::cout<<ModAngle( left - right )<<std::endl;
		return ModAngle( left - right );
	}

	void	TurnPIDReset	()
	{
		turnP			=	0;
		turnI			=	0;
		turnD			=	0;
		turnInterval	=	0;
	}

	/* SENSORY */

	double	zeroLeft	=	0;
	double	zeroRight	=	0;

	double	distLeft	=	0;
	double	distRight	=	0;
	double	distMin		=	0;
	double	posArm		=	0;

	double	speedRight	=	0;
	double	speedLeft	=	0;
	double	speedArm	=	0;

	void	Update	()
	{
		if ( tracking )
		{
			double	_timeElapsed	=	timer.Get();

			distLeft	=	encoderLeft.GetDistance()	-	zeroLeft;
			distRight	=	encoderRight.GetDistance()	-	zeroRight;
			distMin		=	fminf( distLeft , distRight );

			speedLeft	=	encoderLeft.GetRate();
			speedRight	=	encoderLeft.GetRate();

			timer.Reset();
		}
		tracking = true;
	}

	void	ResetEncoders()
	{
		zeroLeft	=	encoderLeft.GetDistance();
		zeroRight	=	encoderRight.GetDistance();
	}

	void	MoveArmDown()
	{
		// raise arm
		while (	armDown.Get()	&&	autoTimer.Get()	<=	3	)	pitch.Set(1);
		// lower arm
		while (	!armDown.Get()	&&	autoTimer.Get()	<=	3	)	pitch.Set(-1);
		// stop
		pitch.Set(0);
	}

	void	SetIntake(float speed)
	{
		intake1.Set(speed);
		intake2.Set(speed);
	}
};

START_ROBOT_CLASS(Robot)
