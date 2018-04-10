package ca.team3161.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import ca.team3161.lib.utils.controls.InvertedJoystickMode;
import ca.team3161.lib.utils.controls.LogitechDualAction;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechAxis;
import ca.team3161.lib.utils.controls.LogitechDualAction.LogitechControl;
import ca.team3161.lib.utils.controls.SquaredJoystickMode;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot
{ 
	private static final double AUTO_PID_LOOP_PERIOD = 0.015;
	private static final int DRIVETRAIN_PEAK_CURRENT_AMP_LIMIT = 35;
	private static final int DRIVETRAIN_PEAK_CURRENT_DURATION = 10;
	private static final int DRIVETRAIN_CONTINUOUS_CURRENT_AMP_LIMIT = 30;
	private static final int ELEVATOR_PEAK_CURRENT_AMP_LIMIT = 15;
	private static final int ELEVATOR_PEAK_CURRENT_DURATION = 10;
	private static final int ELEVATOR_CONTINUOUS_CURRENT_AMP_LIMIT = 10;
	//This is declaring the motors with the corresponding Controllers
	private WPI_TalonSRX frontLeftDrive;
	private WPI_TalonSRX frontRightDrive;
	private WPI_TalonSRX backLeftDrive;
	private WPI_TalonSRX backRightDrive;
	private VictorSP IntakeL = new VictorSP (3);
	private VictorSP IntakeR = new VictorSP (2);
	private VictorSP pivot = new VictorSP (1);
	private WPI_TalonSRX leftElevator = new WPI_TalonSRX (4);
	private VictorSP leftElevatorSlave = new VictorSP(0);
	private WPI_TalonSRX rightElevator = new WPI_TalonSRX (6);
	private DigitalInput bottomLimitSwitch = new DigitalInput(1);

	//Variables to be used when waiting
	boolean isWaiting = false;
	long waitStartTime = -1;

	private int firstAction = 0;
	private int secondAction = 0;
	
	private long timeReachedTarget = -1;

	//Declaring the way the robot will drive - RoboDrive class
	private MecanumDrive drivetrain;

	//Declaring the AHRS class to get gyro headings
	private AHRS ahrs;
	private double angle;

	//Declaring variable to control the end game climb
	private boolean RAISE = true;

	//This is declaring both Controllers 
	private LogitechDualAction driverPad = new LogitechDualAction(0);
	private LogitechDualAction operatorPad = new LogitechDualAction(1);

	//For drive train PID - face buttons
	PIDController turnController;
	double rotate;
	double Pg = 0.02;
	double Ig = 0.00;
	double Dg = 0.06;
	float kToleranceDegrees = 2;
	boolean rotateToAngle;
	double currentRotationRate;

	//For drive train - wheel rotations
	float kToleranceRotations = 100;

	//global PID pos values
	double PX = 0.0006;
	double IX = 0.000001;
	double DX = 0.0022;

	//Front Left Wheel
	PIDController FLController;
	double P0 = PX;
	double I0 = IX;
	double D0 = DX;
	double FLSpeed;
	//Front Right Wheel
	PIDController FRController;
	double P1 = PX;
	double I1 = IX;
	double D1 = DX;
	double FRSpeed;
	//Back Left Wheel
	PIDController BLController;
	double P2 = PX;
	double I2 = IX;
	double D2 = DX;
	double BLSpeed;
	//Back Right Wheel
	PIDController BRController;
	double P3 = PX;
	double I3 = IX;
	double D3 = DX;
	double BRSpeed;
	//A tolerance value that wheel rotations must reach between
	private double tolerance = 350;

	//For elevator PID
	PIDController EController;
	double Pe = 0.0002;
	double Ie = 0.0;
	double De = 0.00003;
	double ESpeed;
	//A value assigned to top, bottom, switch and scale height
	double TOP = -51000;
	double BOTTOM = -5000;
	double SWITCH = 0.0;
	boolean rotateToHeight;

	//Configures all the Joysticks and Buttons for both controllers
	double leftStickX;
	double leftStickY;	 
	double rightStickX;
	double rightStickY;
	double leftStickY_Operator;
	double rightStickY_Operator;
	boolean getButtonSTART;
	boolean getButtonY;
	boolean getButtonX;
	boolean getButtonA;
	boolean getButtonB;
	boolean getButtonRT;
	boolean getButtonRT_Operator;
	boolean getButtonLB_Operator;
	boolean getButtonLT_Operator;
	boolean getButtonY_Operator;
	boolean getButtonX_Operator;
	boolean getButtonA_Operator;
	boolean getButtonB_Operator;
	boolean getButtonRB_Operator;
	boolean getButtonRB;
	boolean elevatorOverride;

	//Need to set variables to use Pneumatics Control Module
	private Compressor air = new Compressor(0);
	private boolean pressureSwitch;

	//Declaring Buttons for the pistons
	private DoubleSolenoid claw = new DoubleSolenoid (1,0);

	//Declaring a string to get the switch/scale positioning for each alliance
	String gameData;

	//Declaring positions for starting autonomous
	private double ticks = 0;

	//Declaring variables to be used in autonomous
	private double YSpeed;
	private double XSpeed;

	//A counter for running the claw's intake after the claw closes
	private int c = 0;

	//A counter that controls operations in autonomous
	private int operation;
	
	Timer autoTimer = new Timer();

	private volatile Thread autoThread = null;

	//objects to help select specific auto paths from the smartdashboard
	private SendableChooser<StartingLocation> autoModeChooser;
	private StartingLocation selectedAutoMode;
	private SendableChooser<Combinations> LL;
	private Combinations fieldOrientations_LL;
	private SendableChooser<Combinations> RR;
	private Combinations  fieldOrientations_RR;
	private SendableChooser<Combinations> LR;
	private Combinations fieldOrientations_LR;
	private SendableChooser<Combinations> RL;
	private Combinations fieldOrientations_RL;

	private enum Combinations {
		one_switch, one_switch_one_scale, one_scale, two_scale, NONE;
	}

	private enum StartingLocation {
		LEFT, MIDDLE, RIGHT, ABORT;
	}

	public void robotInit() 
	{	
		frontLeftDrive = new WPI_TalonSRX(0);
		backLeftDrive = new WPI_TalonSRX(2);
		frontRightDrive = new WPI_TalonSRX(1);
		backRightDrive = new WPI_TalonSRX(3);

		autoModeChooser = new SendableChooser<>();
		LL = new SendableChooser<>();
		RR = new SendableChooser<>();
		LR = new SendableChooser<>();
		RL = new SendableChooser<>();

		//Default Auto Mode  is scoring switch when starting from middle
		autoModeChooser.addDefault("Middle", StartingLocation.MIDDLE);
		autoModeChooser.addObject("Left", StartingLocation.LEFT);
		autoModeChooser.addObject("Right", StartingLocation.RIGHT);
		autoModeChooser.addObject("Abort", StartingLocation.ABORT);
		SmartDashboard.putData("Starting Position", autoModeChooser);

		LL.addDefault("1 Switch, 1 Scale", Combinations.one_switch_one_scale);
		LL.addObject("1 Scale", Combinations.one_scale);
		LL.addObject("2 Scale", Combinations.two_scale);
		LL.addObject("None", Combinations.NONE);
		SmartDashboard.putData("LL Options", LL);

		RR.addDefault("1 Switch, 1 Scale", Combinations.one_switch_one_scale);
		RR.addObject("1 Scale", Combinations.one_scale);
		RR.addObject("2 Scale", Combinations.two_scale);
		RR.addObject("None", Combinations.NONE);
		SmartDashboard.putData("RR Options", RR);

		LR.addDefault("1 Scale", Combinations.one_scale);
		LR.addObject("1 Switch", Combinations.one_switch);
		LR.addObject("2 Scale", Combinations.two_scale);
		LR.addObject("None", Combinations.NONE);
		SmartDashboard.putData("LR Options", LR);

		RL.addDefault("1 Scale", Combinations.one_scale);
		RL.addObject("1 Switch", Combinations.one_switch);
		RL.addObject("2 Scale", Combinations.two_scale);
		RL.addObject("None", Combinations.NONE);
		SmartDashboard.putData("RL Options", RL);

		frontLeftDrive.setInverted(false);
		frontLeftDrive.configPeakCurrentLimit(DRIVETRAIN_PEAK_CURRENT_AMP_LIMIT, 10);
		frontLeftDrive.configPeakCurrentDuration(DRIVETRAIN_PEAK_CURRENT_DURATION, 10);
		frontLeftDrive.configContinuousCurrentLimit(DRIVETRAIN_CONTINUOUS_CURRENT_AMP_LIMIT, 10);
		frontLeftDrive.enableCurrentLimit(true);

		frontRightDrive.setInverted(false);
		frontRightDrive.configPeakCurrentLimit(DRIVETRAIN_PEAK_CURRENT_AMP_LIMIT, 10);
		frontRightDrive.configPeakCurrentDuration(DRIVETRAIN_PEAK_CURRENT_DURATION, 10);
		frontRightDrive.configContinuousCurrentLimit(DRIVETRAIN_CONTINUOUS_CURRENT_AMP_LIMIT, 10);
		frontRightDrive.enableCurrentLimit(true);

		backRightDrive.setInverted(false);
		backRightDrive.configPeakCurrentLimit(DRIVETRAIN_PEAK_CURRENT_AMP_LIMIT, 10);
		backRightDrive.configPeakCurrentDuration(DRIVETRAIN_PEAK_CURRENT_DURATION, 10);
		backRightDrive.configContinuousCurrentLimit(DRIVETRAIN_CONTINUOUS_CURRENT_AMP_LIMIT, 10);
		backRightDrive.enableCurrentLimit(true);

		backLeftDrive.setInverted(false);
		backLeftDrive.configPeakCurrentLimit(DRIVETRAIN_PEAK_CURRENT_AMP_LIMIT, 10);
		backLeftDrive.configPeakCurrentDuration(DRIVETRAIN_PEAK_CURRENT_DURATION, 10);
		backLeftDrive.configContinuousCurrentLimit(DRIVETRAIN_CONTINUOUS_CURRENT_AMP_LIMIT, 10);
		backLeftDrive.enableCurrentLimit(true);

		leftElevator.configPeakCurrentLimit(ELEVATOR_PEAK_CURRENT_AMP_LIMIT, 10);
		leftElevator.configPeakCurrentDuration(ELEVATOR_PEAK_CURRENT_DURATION, 10);
		leftElevator.configContinuousCurrentLimit(ELEVATOR_CONTINUOUS_CURRENT_AMP_LIMIT, 10);
		leftElevator.enableCurrentLimit(true);

		rightElevator.configPeakCurrentLimit(ELEVATOR_PEAK_CURRENT_AMP_LIMIT, 10);
		rightElevator.configPeakCurrentDuration(ELEVATOR_PEAK_CURRENT_DURATION, 10);
		rightElevator.configContinuousCurrentLimit(ELEVATOR_CONTINUOUS_CURRENT_AMP_LIMIT, 10);
		rightElevator.enableCurrentLimit(true);

		//gyro readings for kmxp
		ahrs = new AHRS(SPI.Port.kMXP);
		ahrs.reset();


		//Reads in gyro from I2C connections
		//ahrs = new AHRS(SerialPort.Port.kUSB);

		//Initiate the RoboDrive class, so that drive train variable can be used with the talons - driving controller
		drivetrain = new MecanumDrive(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive);

		//Configuring settings for wheel encoders
		frontLeftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		frontLeftDrive.set(ControlMode.Position, 0);
		frontRightDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		frontRightDrive.set(ControlMode.Position, 0);
		backLeftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		backLeftDrive.set(ControlMode.Position, 0);
		backRightDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		backRightDrive.set(ControlMode.Position, 0);

		//Configuring settings for elevator encoder
		rightElevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		rightElevator.set(ControlMode.Position, 0);

		//Executes PID calculations for gyro face buttons
		turnController = new PIDController(Pg, Ig, Dg, ahrs, this::gyroPidWrite);
		turnController.setInputRange(-180.0f,  180.0f);
		turnController.setOutputRange(-0.8, 0.8);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);		

		final double maxAutoDriveSpeed = 1.0; // 0.75 is known safe

		//Executes PID calculations for wheel rotations - Front Left
		FLController = new PIDController(P0, I0, D0, new TalonSrxPIDSource(frontLeftDrive, 0), this::frontLeftPidWrite);
		FLController.setInputRange(-100000, 100000);
		FLController.setOutputRange(-maxAutoDriveSpeed, maxAutoDriveSpeed);
		FLController.setAbsoluteTolerance(kToleranceRotations);
		FLController.setContinuous(false);

		//Executes PID calculations for wheel rotations - Front Right
		FRController = new PIDController(P1, I1, D1, new TalonSrxPIDSource(frontRightDrive, 0), this::frontRightPidWrite);
		FRController.setInputRange(-100000, 100000);
		FRController.setOutputRange(-maxAutoDriveSpeed, maxAutoDriveSpeed);
		FRController.setAbsoluteTolerance(kToleranceRotations);
		FRController.setContinuous(false);

		//Executes PID calculations for wheel rotations - Back Left
		BLController = new PIDController(P2, I2, D2, new TalonSrxPIDSource(backLeftDrive, 0), this::backLeftPidWrite);
		BLController.setInputRange(-100000, 100000);
		BLController.setOutputRange(-maxAutoDriveSpeed, maxAutoDriveSpeed);
		BLController.setAbsoluteTolerance(kToleranceRotations);
		BLController.setContinuous(false);

		//Executes PID calculations for wheel rotations - Back Right
		BRController = new PIDController(P3, I3, D3, new TalonSrxPIDSource(backRightDrive, 0), this::backRightPidWrite);
		BRController.setInputRange(-100000, 100000);
		BRController.setOutputRange(-maxAutoDriveSpeed, maxAutoDriveSpeed);
		BRController.setAbsoluteTolerance(kToleranceRotations);
		BLController.setContinuous(false);

		//Executes PID calculations for elevator rotations
		EController = new PIDController(Pe, Ie, De, new TalonSrxPIDSource(rightElevator, 0), this::ElevatorPidWrite);
		EController.setInputRange(-100000, 100000);
		EController.setOutputRange(-1.0, 1.0);
		EController.setAbsoluteTolerance(600.0);
		EController.setContinuous(false);

		//WHEN LEFT SIDE WHEELS ROTATE FORWARDS OVER THE TOP, ENCODERS APPROACH -INFINITY
		//WHEN RIGHT SIDE WHEELS ROTATE FORWARDS OVER THE TOP, ENCODERS APPROACH +INFINITY

		//The robot's claw is set to closed - will be holding a cube when enabled
		ClawClose();

		//with proper wiring, calibrates the talons using polarity inversions
		driverPad.setMode(LogitechControl.LEFT_STICK, LogitechAxis.X, new SquaredJoystickMode());
		driverPad.setMode(LogitechControl.LEFT_STICK, LogitechAxis.Y, new InvertedJoystickMode().andThen(new SquaredJoystickMode()));
		driverPad.setMode(LogitechControl.RIGHT_STICK, LogitechAxis.X, new SquaredJoystickMode());
	}

	public void autonomousInit() 
	{
		System.out.println("AUTONOMOUS INIT");
		drivetrain.setSafetyEnabled(false);
		ahrs.reset();
		autoTimer.start();
		//Get scale/switch positions from driver station
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		resetWheelEncoders();
		resetElevatorEncoder();
		//Enables all encoders that're being used in autonomous
		FLController.enable();
		FRController.enable();
		BLController.enable();
		BRController.enable();
		EController.enable();
		//Sets the operation to (0) to run through autonomous commands
		operation = 0;
		//Gets the selected autonomous from smart dashboard
		selectedAutoMode = autoModeChooser.getSelected();
		fieldOrientations_LL = LL.getSelected();
		fieldOrientations_RR = RR.getSelected();
		fieldOrientations_LR = LR.getSelected();
		fieldOrientations_RL = RL.getSelected();

		leftElevator.enableCurrentLimit(true);
		rightElevator.enableCurrentLimit(true);
		
		if (this.autoThread != null) {
			this.autoThread.interrupt();
		}
		
		this.autoThread = new Thread(new Runnable() {
			{
				System.out.println("Auto Thread Initialized");
			}
			@Override
			public void run() {
				while (isEnabled() && isAutonomous()) {
					autonomousRoutine();
					Timer.delay(0.02);
				}
				autoThread = null;
			}
		});
		this.autoThread.start();

	}

	public void autonomousRoutine()
	{
		if (selectedAutoMode.equals(StartingLocation.ABORT)) {
			if(gameData.charAt(0) == 'L')
			{
				if(operation == 0)
				{
					operation += driveForward(1400, 0.0, true, frontRightDrive);
				}
				if(operation == 1)
				{
					Timer.delay(0.3);
					resetWheelEncoders();
					operation++;
				}
				if(operation == 2)
				{
					operation += driveLeft(1600, 0.0, frontRightDrive);
				}
				if(operation == 3)
				{
					operation += driveForward(2800, 0.0, true, frontRightDrive);
				}
				if(operation == 4)
				{
					operation ++;
				}
				if(operation == 5)
				{
					ClawOutput();
					Timer.delay(0.5);
					operation++;
				}
				if(operation == 6)
				{
					ClawStandby();
					operation++;
				}
				if(operation == 7)
				{
					operation += driveBackward(2200, 0.0, false, frontRightDrive);	
				}
				if(operation == 8)
				{
					ClawRotateUp();
					Timer.delay(1.0);
					operation++;
				}
				if(operation == 9)
				{
					ClawOpen();
					operation++;
				}
				if(operation == 10)
				{
					ClawIntake();
					operation += driveRight(1700, 0.0, backRightDrive);
				}
				if(operation == 11)
				{
					operation += driveForward(1700, 0.0, true, frontRightDrive);
				}
				if(operation == 12)
				{
					ClawClose();
					operation++;	
				}
				if(operation == 13)
				{
					operation += waitFor(300);
				}
				if(operation == 14)
				{
					ClawStandby();
					operation++;
				}
				if(operation == 15)
				{
					operation += driveBackward(400, 0.0, true, frontRightDrive);
				}
				if(operation == 16)
				{
					operation += driveLeft(1700, 0.0, frontRightDrive);
				}
				if(operation == 17)
				{
					ClawRotateDown();
					Timer.delay(0.7);
					ClawStop();
					operation++;
				}
				if(operation == 18)
				{
					operation += driveForward(2200, 0.0, true, frontRightDrive);
				}
				if(operation == 19)
				{
					ClawOutput();
				}
			}else if(gameData.charAt(0) == 'R')
			{
				if(operation == 0)
				{
					operation += driveForward(1500, 0.0, true, backLeftDrive);
				}
				if(operation == 1)
				{
					operation += driveRight(2000, 0.0, backRightDrive);
				}
				if(operation == 2)
				{
					operation += driveForward(1600, 0.0, true, backLeftDrive);
				}
				if(operation == 3)
				{
					operation++;
				}
				if(operation == 4)
				{
					ClawOutput();
					Timer.delay(0.5);
					operation++;
				}
				if(operation == 5)
				{
					ClawStandby();
					operation++;
				}
				if(operation == 6)
				{
					operation += driveBackward(1900, 0.0, false, backLeftDrive);	
				}
				if(operation == 7)
				{
					ClawRotateUp();
					Timer.delay(1.0);
					operation++;
				}
				if(operation == 8)
				{
					ClawOpen();
					operation++;
				}
				if(operation == 9)
				{
					ClawIntake();
					operation += driveLeft(2200, 0.0, backRightDrive);
				}
				if(operation == 10)
				{
					operation += driveForward(1700, 0.0, true, backLeftDrive);
				}
				if(operation == 11)
				{
					ClawClose();
					operation++;	
				}
				if(operation == 12)
				{
					Timer.delay(0.3);
					operation ++;
				}
				if(operation == 13)
				{
					ClawStandby();
					operation++;
				}
				if(operation == 14)
				{
					operation += driveBackward(1700, 0.0, true, backLeftDrive);
				}
				if(operation == 15)
				{
					operation += driveRight(2200, 0.0, backLeftDrive);
				}
				if(operation == 16)
				{
					ClawRotateDown();
					Timer.delay(0.7);
					ClawStop();
					operation++;
				}
				if(operation == 17)
				{
					operation += driveForward(2200, 0.0, true, backLeftDrive);
				}
				if(operation == 18)
				{
					ClawOpen();
				}
			}
			return;
		}
		//get air for pneumatics
		/*
		pressureSwitch = air.getPressureSwitchValue();


		if (!pressureSwitch) 
		{
			air.setClosedLoopControl(true);
		}
		 */

		//START OF AUTONOMOUS
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		if(gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L')
		{
			//LL
			switch(selectedAutoMode)
			{
			case LEFT:
				//Start Left - LL
				switch(fieldOrientations_LL)
				{
				case NONE:
					//Drive straight past auto line
					driveStraight();
					break;
				case one_scale:
					//Score Left Scale
					left_LeftScale();
					break;
				case one_switch:
					//Score left Switch
					if(operation == 0)
					{
						operation += driveForward(3000, 0.0, true, backLeftDrive);
					}
					if(operation == 1)
					{
						operation += pidTurnExact(90.0);
					}
					if(operation == 2)
					{
						operation += driveForward(1500, 90.0, false, backLeftDrive);
					}
					if(operation == 3)
					{
						waitFor(500);
						ClawShoot();
						waitFor(500);
						ClawStandby();
					}
					break;
				case one_switch_one_scale:
					//Score Left Scale & Left Switch
					left_LeftScaleLeftSwitch();
					break;
				case two_scale:
					//Score Left Scale x2
					left_TwoLeftScale();
					break;
				default:
					break;
				}

				break;
			case MIDDLE:
				//Start Middle - L Switch
				middle_LeftSwitch();
				break;
			case RIGHT:
				//Start Right - LL
				switch(fieldOrientations_LL)
				{
				case NONE:
					//Drive straight past auto line
					driveStraight();
					break;
				case one_scale:
					//Cross field 1 Cube Scale
					right_LeftScale();
					break;
				case one_switch:
					//Cross field 1 Switch
					break;
				case one_switch_one_scale:
					//Cross field 1 Scale and 1 Switch
					right_LeftScaleLeftSwitch();
					break;
				case two_scale:
					//Cross field 2 Cube
					right_TwoLeftScale();
					break;
				default:
					break;
				}

				break;
			default:
				break;
			}
		}else if(gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R')
		{
			//RR
			switch(selectedAutoMode)
			{
			case LEFT:
				//Start Left - RR
				switch(fieldOrientations_RR)
				{
				case NONE:
					//Drive past auto line
					driveStraight();
					break;
				case one_scale:
					//Cross field 1 Cube Scale
					left_RightScale();
					break;
				case one_switch:
					//Cross field 1 Cube Switch
					break;
				case one_switch_one_scale:
					//Cross field 1 Scale and 1 Switch
					left_RightScaleRightSwitch();
					break;
				case two_scale:
					//Cross field 2 Cube Scale
					left_TwoRightScale();
					break;
				default:
					break;
				}
				break;

			case MIDDLE:
				//Start Middle - Right Switch
				middle_RightSwitch();
				break;
			case RIGHT:
				//Start Right - RR
				switch(fieldOrientations_RR)
				{
				case NONE:
					//Drive past auto line
					driveStraight();
					break;
				case one_scale:
					// Drive up 1 Scale
					right_RightScale();
					break;
				case one_switch:
					//Drive up 1 Switch
					if(operation == 0)
					{
						operation += driveForward(3000, 0.0, true, backLeftDrive);
					}
					if(operation == 1)
					{
						operation += pidTurnExact(-90.0);
					}
					if(operation == 2)
					{
						operation += driveForward(1500, -90.0, false, backRightDrive);
					}
					if(operation == 3)
					{
						waitFor(300);
						ClawShoot();
						waitFor(500);
						ClawStandby();
					}
					break;
				case one_switch_one_scale:
					//Drive up 1 Scale and 1 Switch
					right_RightScaleRightSwitch();
					break;
				case two_scale:
					//Drive up 2 Scale
					right_TwoRightScale();
					break;
				default:
					break;
				}
				break;
			default:
				break;
			}
		}else if(gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R')
		{
			//LR
			switch(selectedAutoMode)
			{
			case LEFT:
				//Starting Left - LR
				switch(fieldOrientations_LR)
				{
				case NONE:
					//Drive past auto line
					driveStraight();
					break;
				case one_scale:
					//Cross field scale
					left_RightScale();
					break;
				case one_switch:
					//Drive up 1 Switch
					if(operation == 0)
					{
						operation += driveForward(3000, 0.0, true, backLeftDrive);
					}
					if(operation == 1)
					{
						operation += pidTurnExact(90.0);
					}
					if(operation == 2)
					{
						operation += driveForward(1500, 90.0, false, backLeftDrive);
					}
					if(operation == 3)
					{
						waitFor(500);
						ClawShoot();
						waitFor(500);
						ClawStandby();
					}
					break;
				case two_scale:
					//Cross field 2 Scale
					left_TwoRightScale();
					break;
				default:
					break;
				}

				break;
			case MIDDLE:
				//Starting Middle - Left Switch
				middle_LeftSwitch();

				break;
			case RIGHT:
				//Starting Right - LR
				switch(fieldOrientations_LR)
				{
				case NONE:
					//Drive past auto line
					driveStraight();
					break;
				case one_scale:
					//Drive up 1 Scale
					right_RightScale();
					break;
				case one_switch:
					//Cross field 1 Switch
					break;
				case two_scale:
					//Drive up 2 Scale
					right_TwoRightScale();
					break;
				default:
					break;
				}
				break;
			default:
				break;
			}
		}else if(gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L')
		{
			//RL
			switch(selectedAutoMode)
			{
			case LEFT:
				//Starting Left - RL
				switch(fieldOrientations_RL)
				{
				case NONE:
					//Drive straight past auto line
					driveStraight();
					break;
				case one_scale:
					//Drive up 1 Scale
					left_LeftScale();
					break;
				case one_switch:
					//Cross field 1 Switch
					break;
				case two_scale:
					//Drive up 2 Scale
					left_TwoLeftScale();
					break;

				default:
					break;
				}

				break;
			case MIDDLE:
				//Starting Middle - Right Switch
				middle_RightSwitch();

				break;
			case RIGHT:
				//Starting Right - RL
				switch(fieldOrientations_RL)
				{
				case NONE:
					//Drive past auto line
					driveStraight();
					break;
				case one_scale:
					//Cross field 1 Scale
					right_LeftScale();
					break;
				case one_switch:
					//Drive up 1 Switch
					if(operation == 0)
					{
						operation += driveForward(3000, 0.0, true, backLeftDrive);
					}
					if(operation == 1)
					{
						operation += pidTurnExact(-90.0);
					}
					if(operation == 2)
					{
						operation += driveForward(1500, -90.0, false, backLeftDrive);
					}
					if(operation == 3)
					{
						waitFor(300);
						ClawShoot();
						waitFor(500);
						ClawStandby();
					}
					break;
				case two_scale:
					//Cross field 2 Scale
					right_TwoLeftScale();
					break;
				default:
					break;
				}
			}
		}

		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//END OF AUTONOMOUS

		//stop taking air when pneumatics reaches 120 psi
		/*
		if (pressureSwitch) 
		{
			air.setClosedLoopControl(false);
		}
		showDisplay();
		 /*
		 */

		drivetrain.stopMotor();

		showDisplay();
	}

	public void teleopInit()
	{
		drivetrain.setSafetyEnabled(true);
		resetWheelEncoders();
		FLController.disable();
		FRController.disable();
		BLController.disable();
		BRController.disable();
		leftElevator.enableCurrentLimit(true);
		rightElevator.enableCurrentLimit(true);
	}

	public void teleopPeriodic() 
	{
		if (Timer.getMatchTime() < 30) {
			leftElevator.enableCurrentLimit(false);
			rightElevator.enableCurrentLimit(false);
		}

		showDisplay();

		leftStickX = driverPad.getValue(LogitechDualAction.LogitechControl.LEFT_STICK, LogitechDualAction.LogitechAxis.X);
		leftStickY = driverPad.getValue(LogitechDualAction.LogitechControl.LEFT_STICK, LogitechDualAction.LogitechAxis.Y);
		rightStickX = driverPad.getValue(LogitechDualAction.LogitechControl.RIGHT_STICK, LogitechDualAction.LogitechAxis.X);
		getButtonSTART = driverPad.getButton(LogitechDualAction.LogitechButton.START);
		getButtonY = driverPad.getButton(LogitechDualAction.LogitechButton.Y);
		getButtonX = driverPad.getButton(LogitechDualAction.LogitechButton.X);
		getButtonA = driverPad.getButton(LogitechDualAction.LogitechButton.A);
		getButtonRT_Operator = operatorPad.getButton(LogitechDualAction.LogitechButton.RIGHT_TRIGGER);
		getButtonRT = driverPad.getButton(LogitechDualAction.LogitechButton.RIGHT_TRIGGER);
		getButtonB = driverPad.getButton(LogitechDualAction.LogitechButton.B);
		getButtonLB_Operator = operatorPad.getButton(LogitechDualAction.LogitechButton.LEFT_BUMPER);
		getButtonLT_Operator = operatorPad.getButton(LogitechDualAction.LogitechButton.LEFT_TRIGGER);
		leftStickY_Operator = operatorPad.getValue(LogitechDualAction.LogitechControl.LEFT_STICK, LogitechDualAction.LogitechAxis.Y);
		rightStickY_Operator = operatorPad.getValue(LogitechDualAction.LogitechControl.RIGHT_STICK, LogitechDualAction.LogitechAxis.Y );
		getButtonY_Operator = operatorPad.getButton(LogitechDualAction.LogitechButton.Y);
		getButtonX_Operator = operatorPad.getButton(LogitechDualAction.LogitechButton.X);
		getButtonA_Operator = operatorPad.getButton(LogitechDualAction.LogitechButton.A);
		getButtonB_Operator = operatorPad.getButton(LogitechDualAction.LogitechButton.B);
		getButtonRB = driverPad.getButton(LogitechDualAction.LogitechButton.RIGHT_BUMPER);
		elevatorOverride = operatorPad.getButton(LogitechDualAction.LogitechButton.SELECT);

		//gets yaw from gyro continuously
		angle = ahrs.getYaw();

		//get air for pneumatics
		pressureSwitch = air.getPressureSwitchValue();

		if (!pressureSwitch) 
		{
			air.setClosedLoopControl(true);
		}

		//Dead band, restricts stick movement when less than 5% for all controls
		if (Math.abs(rightStickX) < 0.05)
		{
			rightStickX = 0;
		}
		if (Math.abs(leftStickY) < 0.05) 
		{
			leftStickY = 0;
		}
		if (Math.abs(leftStickX) < 0.05) 
		{
			leftStickX = 0;
		}

		//Preset angles for the robot - to be called with buttons A, B, X, Y
		rotateToAngle = false;
		if (getButtonSTART) 
		{
			ahrs.reset();
		}
		//Look Forward
		if (getButtonY) 
		{
			currentRotationRate = gyroPID(0.0);
			rotateToAngle = true;
		}
		//Look Right
		else if (getButtonB) 
		{
			currentRotationRate = gyroPID(90.0);
			rotateToAngle = true;
		}
		//Look Backward
		else if (getButtonA) 
		{
			currentRotationRate = gyroPID(180.0);
			rotateToAngle = true;
		}
		//Look Left
		else if (getButtonX) 
		{
			currentRotationRate = gyroPID(-90.0);
			rotateToAngle = true;
		}

		if(!rotateToAngle)
		{
			turnController.disable();
			currentRotationRate = rightStickX;
		}

		//If pressed, all robot movements are halved - To not tip over
		if(getButtonRT)
		{
			leftStickX *= 0.5;
			leftStickY *= 0.5;
			currentRotationRate *= 0.5;
		}

		//Setting speeds for the claw's motors to intake or shoot out the cube
		//Left Bumper intakes, Left Trigger spits out cube
		if(getButtonLT_Operator)
		{
			ClawOutput();
		}
		else if(getButtonLB_Operator)
		{
			ClawShoot();
		}else
		{
			ClawStandby();
		}

		//Setting positions for the pistons
		//Pressing Right Trigger opens the claw, claw is closed when released
		if(getButtonRT_Operator)
		{
			c = 0;
			ClawOpen();
			ClawIntake();
		}else
		{
			c++;
			//Runs the ClawIntake for a bit after the claw closes, to ensure we have the cube
			if(c <= 40)
			{
				ClawIntake();
			}
			if(c == 100000)
			{
				c = 0;
			}
			ClawClose();
		}

		if (elevatorOverride || !bottomLimitSwitch.get()) {
			resetElevatorEncoder();
		}		

		//Move the elevator up or down
		if (leftStickY_Operator > 0.25 && (elevatorOverride || (bottomLimitSwitch.get())))
		{
			ElevatorDown();
			leftStickX *= 0.85;
			leftStickY *= 0.85;
			currentRotationRate *= 0.5;
		}else if (leftStickY_Operator < -0.25 && !(elevatorOverride))
		{
			ElevatorUp();
		}
		else 
		{
			ElevatorHold();
		}

		//Elevator preset buttons
		if (getButtonY_Operator) {
			EController.enable();
			elevatorPosition(52000);
			leftStickX *= 0.5;
			leftStickY *= 0.5;
			currentRotationRate *= 0.5;
		}
		else if (getButtonX_Operator) {
			EController.enable();
			elevatorPosition(25000);
		}
		else if (getButtonB_Operator) {
			EController.enable();
			elevatorPosition(15000);
		}
		else if (getButtonA_Operator) {
			EController.enable();
			elevatorPosition(800);
		}
		else {
			EController.disable();
		}

		//Right stick on operator controller raises/lowers claw based on input - otherwise it holds
		ClawRotate();

		//Stop taking air when pneumatics reaches 120 psi
		if (pressureSwitch == true) 
		{
			air.setClosedLoopControl(false);
		}
		//Calls upon the mecanumDrive_Cartesian method that sends specific power to the talons
		drivetrain.driveCartesian (leftStickX, leftStickY, currentRotationRate * 0.75, -angle);
	}

	@Override
	public void disabledInit() {
		drivetrain.setSafetyEnabled(true);
		FLController.disable();
		FRController.disable();
		BLController.disable();
		BRController.disable();
		turnController.disable();
		EController.disable();
	}

	//Receives gyro PID values
	public void gyroPidWrite(double output) 
	{
		rotate = output;
	}

	//Receives front left motor PID values 
	public void frontLeftPidWrite(double speed0)
	{
		FLSpeed = speed0;
	}

	//Receives front right motor PID values 
	public void frontRightPidWrite(double speed1)
	{
		FRSpeed = speed1;
	}

	//Receives back left motor PID values 
	public void backLeftPidWrite(double speed2)
	{
		BLSpeed = speed2;
	}

	//Receives back right motor PID values 
	public void backRightPidWrite(double speed3)
	{
		BRSpeed = speed3;
	}

	//Receives right elevator PID values
	public void ElevatorPidWrite(double speed4)
	{
		ESpeed = -speed4;
	}

	//Sets the gyro to make the robot face a cetain angle
	private double gyroPID(double angle)
	{
		turnController.setSetpoint(angle);
		turnController.enable();
		return rotate;
	}

	//Intakes the cube
	private void ClawIntake()
	{
		IntakeL.set(0.7);
		IntakeR.set(-0.7);
	}

	//Spits out the cube
	private void ClawOutput()
	{
		IntakeL.set(-0.5);
		IntakeR.set(0.5);
	}

	//Gives the operator an option to shoot the cube farther distances
	private void ClawShoot()
	{
		IntakeL.set(-1.0);
		IntakeR.set(1.0);
	}

	//Motors pulling cube in lightly to prevent losing the cube 
	private void ClawStandby()
	{
		IntakeL.set(0.12);
		IntakeR.set(-0.12);
	}

	//Close claw
	private void ClawClose()
	{
		claw.set(DoubleSolenoid.Value.kReverse);	
	} 

	//Open claw
	private void ClawOpen()
	{
		claw.set(DoubleSolenoid.Value.kForward);
	}
	//Elevator Up
	private void ElevatorUp()
	{
		leftElevator.set(1);
		leftElevatorSlave.set(1);
		rightElevator.set(1);
	}
	//Elevator Down
	private void ElevatorDown()
	{
		leftElevator.set(-1.0);
		leftElevatorSlave.set(-1.0);
		rightElevator.set(-1.0);
	}

	private void ElevatorHold()
	{
		leftElevator.set(0.0);
		leftElevatorSlave.set(0.0);
		rightElevator.set(0.0);
	}
	//Claw Rotate Up
	private void ClawRotateUp()
	{
		pivot.set(-0.8);
	}

	//Claw Rotate Down
	private void ClawRotateDown()
	{
		pivot.set(0.8);
	}
	
	private void ClawRotate()
	{
		pivot.set(rightStickY_Operator);
	}

	//Claw motors are stopped
	private void ClawStop()
	{
		pivot.set(0.0);
	}

	private int pidTurnExact(double angle)
	{
		turnController.setSetpoint(angle);
		turnController.enable();

		while (isEnabled() && isAutonomous() &&
				!(ahrs.getYaw() >= angle - kToleranceDegrees && ahrs.getYaw() <= angle + kToleranceDegrees)) {
			double currentAngle = ahrs.getYaw();
			drivetrain.driveCartesian(0.0, 0.0, rotate, currentAngle);
			Timer.delay(AUTO_PID_LOOP_PERIOD);
		}
		resetWheelEncoders();
		drivetrain.stopMotor();
		turnController.disable();
		return isAutonomous() && isEnabled() ? 1 : 0;

	}

	//Getting elevator to desired height
	private int elevatorPosition(double position)
	{
		EController.setSetpoint(-position);
		EController.enable();
		rightElevator.set(ESpeed);
		leftElevator.set(ESpeed);
		leftElevatorSlave.set(ESpeed);

		if(getTicks(rightElevator) >= -position - 4000 && getTicks(rightElevator) <= -position + 4000)
		{
			EController.disable();
			return 1;
		}else
		{
			return 0;
		}
	}

	private int driveForward(double ticks, double driveAngle, boolean fieldCentric, WPI_TalonSRX talon)
	{
		resetWheelEncoders();
		//FRONT LEFT
		FLController.setSetpoint(ticks);
		FLController.enable();

		//FRONT RIGHT
		FRController.setSetpoint(-ticks);
		FRController.enable();

		//BACK LEFT
		BLController.setSetpoint(ticks);
		BLController.enable();

		//BACK RIGHT
		BRController.setSetpoint(-ticks);
		BRController.enable();
		
		double currentTicks = getTicks(talon);
		long now = System.currentTimeMillis();
		boolean onTarget = Math.abs(currentTicks) >= ticks - tolerance && Math.abs(currentTicks) <= ticks + tolerance;
		boolean timeOnTargetElapsed = timeReachedTarget > 0 && (now > timeReachedTarget + 500);
		

		while (isEnabled() && isAutonomous() &&
				(!onTarget || !timeOnTargetElapsed)) {
			
			if(onTarget) {
				if(timeReachedTarget <= 0) {
					timeReachedTarget = now;
				}
			}else {
				timeReachedTarget = -1;
			}
			
			currentTicks = getTicks(talon);
			now = System.currentTimeMillis();
			onTarget = Math.abs(currentTicks) >= ticks - tolerance && Math.abs(currentTicks) <= ticks + tolerance;
			timeOnTargetElapsed = timeReachedTarget > 0 && (now > timeReachedTarget + 500);
			
			YSpeed = (FLSpeed + BLSpeed + FRSpeed + BRSpeed) / 4;
			//Getting current angle and speed to use in driveCartesian()
			angle = ahrs.getYaw();
			drivetrain.driveCartesian(-0.09, YSpeed, gyroPID(driveAngle), fieldCentric ? -angle : 0);
			
			showDisplay();
			Timer.delay(AUTO_PID_LOOP_PERIOD);
		}
		resetWheelEncoders();
		drivetrain.stopMotor();
		FLController.disable();
		FRController.disable();
		BLController.disable();
		BRController.disable();
		return isAutonomous() && isEnabled() ? 1 : 0;
	}

	//Driving left using encoders
	private int driveRight(double ticks, double driveAngle, WPI_TalonSRX talon)
	{
		resetWheelEncoders();
		//FRONT LEFT
		FLController.setSetpoint(ticks);
		FLController.enable();

		//FRONT RIGHT
		FRController.setSetpoint(ticks);
		FRController.enable();

		//BACK LEFT
		BLController.setSetpoint(-ticks);
		BLController.enable();

		//BACK RIGHT
		BRController.setSetpoint(-ticks);
		BRController.enable();
		
		double currentTicks = getTicks(talon);
		long now = System.currentTimeMillis();
		boolean onTarget = Math.abs(currentTicks) >= ticks - tolerance && Math.abs(currentTicks) <= ticks + tolerance;
		boolean timeOnTargetElapsed = timeReachedTarget > 0 && (now > timeReachedTarget + 500);

		while (isEnabled() && isAutonomous() &&
				(!onTarget || !timeOnTargetElapsed)) {
			
			if(onTarget) {
				if(timeReachedTarget <= 0) {
					timeReachedTarget = now;
				}
			}else {
				timeReachedTarget = -1;
			}
			
			currentTicks = getTicks(talon);
			now = System.currentTimeMillis();
			onTarget = Math.abs(currentTicks) >= ticks - tolerance && Math.abs(currentTicks) <= ticks + tolerance;
			timeOnTargetElapsed = timeReachedTarget > 0 && (now > timeReachedTarget + 500);
			
			XSpeed = (FLSpeed + BLSpeed + FRSpeed + BRSpeed) / 4;
			angle = ahrs.getYaw();
			drivetrain.driveCartesian(XSpeed, 0.0, gyroPID(driveAngle), -angle);
			
			showDisplay();
			Timer.delay(AUTO_PID_LOOP_PERIOD);
		}
		resetWheelEncoders();
		drivetrain.stopMotor();
		FLController.disable();
		FRController.disable();
		BLController.disable();
		BRController.disable();
		return isAutonomous() && isEnabled() ? 1 : 0;	
	}

	//Driving right using encoders
	private int driveLeft(double ticks, double driveAngle, WPI_TalonSRX talon)
	{
		resetWheelEncoders();
		//FRONT LEFT
		FLController.setSetpoint(-ticks);
		FLController.enable();

		//FRONT RIGHT
		FRController.setSetpoint(-ticks);
		FRController.enable();

		//BACK LEFT
		BLController.setSetpoint(ticks);
		BLController.enable();

		//BACK RIGHT
		BRController.setSetpoint(ticks);
		BRController.enable();
		
		double currentTicks = getTicks(talon);
		long now = System.currentTimeMillis();
		boolean onTarget = Math.abs(currentTicks) >= ticks - tolerance && Math.abs(currentTicks) <= ticks + tolerance;
		boolean timeOnTargetElapsed = timeReachedTarget > 0 && (now > timeReachedTarget + 500);

		while (isEnabled() && isAutonomous() &&
				(!onTarget || !timeOnTargetElapsed)) {
			
			if(onTarget) {
				if(timeReachedTarget <= 0) {
					timeReachedTarget = now;
				}
			}else {
				timeReachedTarget = -1;
			}
			
			currentTicks = getTicks(talon);
			now = System.currentTimeMillis();
			onTarget = Math.abs(currentTicks) >= ticks - tolerance && Math.abs(currentTicks) <= ticks + tolerance;
			timeOnTargetElapsed = timeReachedTarget > 0 && (now > timeReachedTarget + 500);
			
			XSpeed = (FLSpeed + BLSpeed + FRSpeed + BRSpeed) / 4;
			angle = ahrs.getYaw();
			drivetrain.driveCartesian(XSpeed, 0.0, gyroPID(driveAngle), -angle);
			
			showDisplay();
			Timer.delay(AUTO_PID_LOOP_PERIOD);
		}
		resetWheelEncoders();
		drivetrain.stopMotor();
		FLController.disable();
		FRController.disable();
		BLController.disable();
		BRController.disable();
		return isAutonomous() && isEnabled() ? 1 : 0;
	}

	//Driving backward using encoders
	private int driveBackward(double ticks, double driveAngle, boolean fieldCentric, WPI_TalonSRX talon)
	{
		resetWheelEncoders();
		//FRONT LEFT
		FLController.setSetpoint(-ticks);
		FLController.enable();

		//FRONT RIGHT
		FRController.setSetpoint(ticks);
		FRController.enable();

		//BACK LEFT
		BLController.setSetpoint(-ticks);
		BLController.enable();

		//BACK RIGHT
		BRController.setSetpoint(ticks);
		BRController.enable();
		
		double currentTicks = getTicks(talon);
		long now = System.currentTimeMillis();
		boolean onTarget = Math.abs(currentTicks) >= ticks - tolerance && Math.abs(currentTicks) <= ticks + tolerance;
		boolean timeOnTargetElapsed = timeReachedTarget > 0 && (now > timeReachedTarget + 500);

		while (isEnabled() && isAutonomous() &&
				(!onTarget || !timeOnTargetElapsed)) {
			
			if(onTarget) {
				if(timeReachedTarget <= 0) {
					timeReachedTarget = now;
				}
			}else {
				timeReachedTarget = -1;
			}
			
			currentTicks = getTicks(talon);
			now = System.currentTimeMillis();
			onTarget = Math.abs(currentTicks) >= ticks - tolerance && Math.abs(currentTicks) <= ticks + tolerance;
			timeOnTargetElapsed = timeReachedTarget > 0 && (now > timeReachedTarget + 500);			
			
			YSpeed = (FLSpeed + BLSpeed + FRSpeed + BRSpeed) / 4;
			angle = ahrs.getYaw();
			drivetrain.driveCartesian(0, YSpeed, gyroPID(driveAngle), fieldCentric ? -angle :0);
			
			showDisplay();
			Timer.delay(AUTO_PID_LOOP_PERIOD);
		}
		Timer.delay(0.4);
		resetWheelEncoders();
		drivetrain.stopMotor();
		FLController.disable();
		FRController.disable();
		BLController.disable();
		BRController.disable();
		return isAutonomous() && isEnabled() ? 1 : 0;
	}

	//Resets all encoders
	private void resetWheelEncoders()
	{
		frontLeftDrive.setSelectedSensorPosition(0, 0, 0);
		frontRightDrive.setSelectedSensorPosition(0, 0, 0);
		backLeftDrive.setSelectedSensorPosition(0, 0, 0);
		backRightDrive.setSelectedSensorPosition(0, 0, 0);
	}

	//Resets elevator encoder
	private void resetElevatorEncoder()
	{
		rightElevator.setSelectedSensorPosition(0, 0, 0);
	}

	//Returns the tick count from encoders
	private double getTicks(WPI_TalonSRX talon)
	{
		return talon.getSelectedSensorPosition(0);
	}

	//Returns velocity from encoders
	private double getVelocity(WPI_TalonSRX talon)
	{
		return talon.getSelectedSensorVelocity(0);
	}

	//Displays information on smart dash board
	private void showDisplay()
	{
		SmartDashboard.putNumber("Front Left Encoder:", frontLeftDrive.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Front Right Encoder:", frontRightDrive.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Back Left Encoder:", backLeftDrive.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Back Right Encoder:", backRightDrive.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Velocity0:", frontLeftDrive.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Velocity1:", frontRightDrive.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Velocity2:", backLeftDrive.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Velocity3:", backRightDrive.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Gyro:", ahrs.getYaw());
		SmartDashboard.putBoolean("Claw Open:", getButtonRT_Operator);
		SmartDashboard.putNumber("Elevator Height", rightElevator.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Operation", operation);
		SmartDashboard.putNumber("ESPEED:", ESpeed);
		SmartDashboard.putNumber("YSPEED", YSpeed);
	}

	private int waitFor(int millis)
	{
		long now = System.currentTimeMillis();
		if(isWaiting && (now >= waitStartTime + millis))
		{
			waitStartTime = -1;
			isWaiting = false;
			return 1;
		}
		if(isWaiting)
		{
			return 0;
		}else
		{
			isWaiting = true;
			waitStartTime = now;
			return 0;
		}
	}

	//Just drive past auto line
	private void driveStraight()
	{
		if(operation == 0)
		{
			operation += driveForward(4000, 0.0, true, backLeftDrive);
		}
		return;
	}

	//Start Middle, score left switch
	private void middle_LeftSwitch() 
	{
		if(operation == 0)
		{
			operation += driveForward(2100, 0.0, true, backLeftDrive);
		}
		if(operation == 1)
		{
			operation++;
		}
		if(operation == 2)
		{
			operation += driveLeft(1250, 0.0, backRightDrive);
		}
		if(operation == 3)
		{
			operation += driveForward(2500, 0.0, true, frontRightDrive);
		}
		if(operation == 4)
		{
			operation ++;
		}
		if(operation == 5)
		{
			operation++;
		}
		if(operation == 6)
		{
			Timer.delay(0.30);
			ClawOutput();
			operation++;
		}
		if (operation == 7)
		{
			Timer.delay(0.5);
			ClawStandby();	
			operation++;
		}
	}

	//Start Middle, score right switch
	private void middle_RightSwitch()
	{
		if(operation == 0)
		{
			operation += driveForward(1500, 0.0, true, backLeftDrive);
		}
		if(operation == 1)
		{
			operation += driveRight(2200, 0.0, backRightDrive);
		}
		if(operation == 2)
		{
			operation += driveForward(2200, 0.0, true, backLeftDrive);
		}
		if(operation == 3)
		{
			operation ++;
		}
		if(operation == 4)
		{
			operation ++;
		}
		if(operation == 5)
		{
			Timer.delay(0.3);
			operation++;
		}
		if(operation == 6)
		{
			ClawOutput();
			Timer.delay(0.5);
			operation++;
		}
		if(operation == 7)
		{
			ClawStandby();
			operation++;
		}

	}

	//Start Left, left scale
	private void left_LeftScale()
	{
		if(operation == 0)
		{
			operation += driveForward(7800, 0.0, true, backLeftDrive);
		}
		if(operation == 1)
		{
			operation += pidTurnExact(60.0);
		}
		if(operation == 2)
		{
			operation += elevatorPosition(35000);
		}
		if(operation == 3)
		{
			operation ++;
		}
		if(operation == 4) 
		{
			ClawRotateUp();
			Timer.delay(0.075);
			operation++;
		}
		if(operation == 5) {	
			ClawStop();
			operation++;
		}
		if(operation == 6) {
			Timer.delay(0.3);
			operation++;
		}
		if (operation == 7) {
			ClawOutput();
			Timer.delay(0.5);
			operation++;
		}
		if (operation == 8) {
			ClawStandby();
			Timer.delay(0.3);
			operation++;
		}
		if(operation == 9)
		{
			operation += elevatorPosition(1000);

		}
	}

	private void left_RightScaleRightSwitch()
	{
		if(operation == 0)
		{
			operation += driveForward(6100, 0.0, true, backLeftDrive);
		}
		if(operation == 1)
		{
			operation += driveRight(7700, 0.0, backLeftDrive);
		}
		if(operation == 2)
		{
			operation++;
		}
		if(operation == 3)
		{
			operation += driveForward(1000, 0.0, true, backLeftDrive);
		}
		if(operation == 4)
		{
			operation += pidTurnExact(-60.0);
		}
		if(operation == 5)
		{
			operation += elevatorPosition(35000);
		}
		if(operation == 6)
		{
			ClawRotateUp();
			Timer.delay(0.075);
			operation++;
		}
		if (operation == 7) {
			ClawStop();
			Timer.delay(0.1);
			operation++;
		}
		if (operation == 8) {
			ClawShoot();
			Timer.delay(0.3);
			operation++;
		}
		if (operation == 9) {
			ClawStandby();
			Timer.delay(0.3);
			operation++;
		}
		if(operation == 10)
		{
			operation += elevatorPosition(1000);

		}
		if(operation == 11)
		{
			operation += pidTurnExact(-150);
		}
		if(operation == 12)
		{
			ClawRotateUp();
			Timer.delay(0.5);
			operation++;
		}
		if(operation == 13)
		{
			ClawStop();
			ClawOpen();
			ClawIntake();
			Timer.delay(0.3);
			operation++;
		}
		if(operation == 14)
		{
			operation += driveForward(2100, -150.0, false, backLeftDrive);
		}
		if(operation == 15)
		{
			ClawClose();
			ClawStandby();
			Timer.delay(1.0);
			operation++;
		}
		if(operation == 16)
		{
			operation += elevatorPosition(30000);
		}
		if(operation == 17)
		{
			operation += driveForward(1000, -150.0, false, backLeftDrive);
		}
		if(operation == 18)
		{
			ClawOpen();
			operation++;
		}
	}

	private void left_LeftScaleLeftSwitch()
	{
		if(operation == 0)
		{
			operation += driveForward(7000, 0.0, true, backLeftDrive);
		}
		if(operation == 1)
		{
			operation += pidTurnExact(60.0);
		}
		if(operation == 2)
		{
			operation += elevatorPosition(35000);
		}
		if(operation == 3)
		{
			ClawRotateUp();
			Timer.delay(0.075);
			operation++;
		}
		if(operation == 4)
		{
			ClawStop();
			Timer.delay(0.1);
			operation++;
		}
		if(operation == 5)
		{
			ClawOutput();
			Timer.delay(0.1);
			operation++;
		}
		if(operation == 6)
		{
			operation += elevatorPosition(1000);
		}
		if(operation == 7)
		{
			operation += pidTurnExact(150);
		}
		if(operation == 8)
		{
			ClawRotateUp();
			Timer.delay(0.5);
			operation++;
		}
		if(operation == 9)
		{
			ClawStop();
			ClawOpen();
			ClawIntake();
			operation++;
		}
		if(operation == 10)
		{
			operation += driveForward(2400, 150.0, false, backLeftDrive);
		}
		if(operation == 11)
		{
			ClawClose();
			Timer.delay(1.0);
			operation++;
		}
		if(operation == 12)
		{
			ClawStandby();
			Timer.delay(0.3);
			operation++;
		}

		if(operation == 13)
		{
			operation += elevatorPosition(30000);
		}
		if(operation == 14)
		{
			operation += driveForward(1000, 150, false, backLeftDrive);
		}
		if(operation == 15)
		{
			ClawOpen();
			operation++;
		}
	}

	private void left_TwoLeftScale()
	{
		if(operation == 0)
		{
			operation += driveForward(7500, 0.0, true, backLeftDrive);
		}
		if(operation == 1)
		{
			operation += pidTurnExact(60.0);
		}
		if(operation == 2)
		{
			operation += elevatorPosition(35000);
		}
		if(operation == 3)
		{
			ClawRotateUp();
			Timer.delay(0.075);
			operation++;
		}
		if(operation == 4)
		{
			ClawStop();
			Timer.delay(0.1);
			operation++;
		}
		if(operation == 5)
		{
			ClawOutput();
			Timer.delay(0.3);
			operation++;
		}
		if(operation == 6)
		{
			ClawStandby();
			operation++;
		}
		if(operation == 7)
		{
			operation += elevatorPosition(1000);

		}
		if(operation == 8)
		{
			operation += pidTurnExact(150);
		}
		if(operation == 9)
		{
			ClawRotateUp();
			Timer.delay(0.5);
			operation++;
		}
		if(operation == 10)
		{
			ClawStop();
			ClawOpen();
			ClawIntake();
			operation++;
		}
		if(operation == 11)
		{
			operation += driveForward(2400, 150.0, false, backLeftDrive);
		}
		if(operation == 12)
		{
			ClawClose();
			Timer.delay(0.3);
			operation++;
		}
		if(operation == 13)
		{
			ClawStandby();
			operation++;
		}
		if(operation == 14)
		{
			operation += pidTurnExact(-30);
		}
		if(operation == 15)
		{
			operation += driveForward(2000, -30, false, backLeftDrive);
		}
		if(operation == 16)
		{
			operation += pidTurnExact(0.0);
		}
		if(operation == 17)
		{
			operation += elevatorPosition(45000);
		}
		if(operation == 18)
		{
			operation += driveForward(300, 0.0, true, backLeftDrive);
		}
		if(operation == 19)
		{
			ClawOutput();
			Timer.delay(0.5);
			operation++;
		}
		if(operation == 20)
		{
			ClawStandby();
			operation++;
		}
	}

	//Start Left, right scale
	private void left_RightScale()
	{
		if(operation == 0)
		{
			operation += driveForward(6200, 0.0, true, backLeftDrive);
		}
		if(operation == 1)
		{
			operation += driveRight(7900, 0.0, backLeftDrive);
		}
		if(operation == 2)
		{
			operation += driveForward(1500, 0.0, true, backLeftDrive);
		}
		if(operation == 3)
		{
			operation += pidTurnExact(-60.0);
		}
		if(operation == 4)
		{
			operation += elevatorPosition(35000);
		}
		if(operation == 5)
		{
			ClawRotateUp();
			Timer.delay(0.075);
			operation++;
		}
		if(operation == 6)
		{
			ClawStop();
			Timer.delay(0.1);
			operation++;
		}
		if(operation == 7)
		{
			ClawOutput();
			Timer.delay(0.5);
			operation++;
		}
		if(operation == 8)
		{
			ClawStandby();
			operation++;
		}
		if(operation == 7)
		{
			operation += elevatorPosition(1000);

		}
	}

	//Start Left, Right Scale x2
	private void left_TwoRightScale()
	{
		if(operation == 0)
		{
			operation += driveForward(6200, 0.0, true, backLeftDrive);
		}
		if(operation == 1)
		{
			operation += driveRight(7700, 0.0, backLeftDrive);
		}
		if(operation == 2)
		{
			operation += driveForward(1500, 0.0, true, backLeftDrive);
		}
		if(operation == 3)
		{
			operation += pidTurnExact(-60.0);
		}
		if(operation == 4)
		{
			operation += elevatorPosition(35000);
		}
		if(operation == 5)
		{
			ClawRotateUp();
			Timer.delay(0.075);
			operation++;
		}
		if(operation == 6)
		{
			ClawStop();
			Timer.delay(0.1);
			operation++;
		}
		if(operation == 7)
		{
			ClawShoot();
			Timer.delay(0.5);
		}
		if(operation == 8)
		{
			ClawStandby();
			operation++;
		}
		if(operation == 7)
		{
			operation += elevatorPosition(1000);
		}
		if(operation == 8)
		{
			operation += pidTurnExact(-150);
		}
		if(operation == 9)
		{
			ClawRotateUp();
			Timer.delay(0.5);
			operation++;
		}
		if(operation == 10)
		{
			ClawStop();
			ClawOpen();
			ClawIntake();
			operation++;
		}
		if(operation == 10)
		{
			operation += driveForward(2100, -150.0, false, backLeftDrive);
		}
		if(operation == 11)
		{
			ClawClose();
			Timer.delay(0.3);
			operation++;
		}
		if(operation == 12)
		{
			ClawStandby();
			operation++;
		}
		if(operation == 13)
		{
			operation += pidTurnExact(30);
		}
		if(operation == 14)
		{
			operation += driveForward(2000, 30, false, backLeftDrive);
		}
		if(operation == 15)
		{
			operation += pidTurnExact(0.0);
		}
		if(operation == 16)
		{
			operation += elevatorPosition(52000);
		}
		if(operation == 17)
		{
			operation += driveForward(300, 0.0, true, backLeftDrive);
		}
		if(operation == 18)
		{
			ClawOutput();
			Timer.delay(0.5);
			operation++;
		}
		if(operation == 19)
		{
			ClawStandby();
			operation++;
		}
	}

	private void right_LeftScale()
	{
		if(operation == 0)
		{
			operation += driveForward(6200, 0.0, true, frontRightDrive);
		}
		if(operation == 1)
		{
			operation += driveLeft(7900, 0.0, backLeftDrive);
		}
		if(operation == 2)
		{
			operation += driveForward(2500, 0.0, true, frontRightDrive);
		}
		if(operation == 3)
		{
			operation += pidTurnExact(60.0);
		}
		if(operation == 4)
		{
			operation += elevatorPosition(35000);
		}
		if(operation == 5)
		{
			ClawRotateUp();
			Timer.delay(0.075);
			operation++;
		}
		if(operation == 6)
		{
			ClawStop();
			Timer.delay(0.3);
			operation++;
		}
		if(operation == 7)
		{
			ClawOutput();
			Timer.delay(0.5);
			operation++;
		}
		if(operation == 8)
		{
			ClawStandby();
			operation++;
		}
		if(operation == 9)
		{
			operation += elevatorPosition(1000);
		}
	}

	private void right_TwoLeftScale()
	{
		if(operation == 0)
		{
			operation += driveForward(6100, 0.0, true, frontRightDrive);
		}
		if(operation == 1)
		{
			operation += driveLeft(7700, 0.0, backLeftDrive);
		}
		if(operation == 2)
		{
			operation += driveForward(2000, 0.0, true, frontRightDrive);
		}
		if(operation == 3)
		{
			operation += pidTurnExact(60.0);
		}
		if(operation == 4)
		{
			operation += elevatorPosition(35000);
		}
		if(operation == 5)
		{
			ClawRotateUp();
			Timer.delay(0.075);
			operation++;
		}
		if(operation == 6)
		{
			ClawStop();
			Timer.delay(0.3);
			operation++;
		}
		if(operation == 7)
		{
			ClawOutput();
			Timer.delay(0.5);
			operation++;
		}
		if(operation == 8)
		{
			ClawStandby();
			operation++;
		}
		if(operation == 9)
		{
			operation += elevatorPosition(1000);
		}
		if(operation == 10)
		{
			operation += pidTurnExact(150);
		}
		if(operation == 11)
		{
			ClawRotateUp();
			Timer.delay(0.5);
			operation++;
		}
		if(operation == 12)
		{
			ClawStop();
			ClawOpen();
			ClawIntake();
			operation++;
		}
		if(operation == 13)
		{
			operation += driveForward(2100, 150.0, false, frontRightDrive);
		}
		if(operation == 14)
		{
			ClawClose();
			Timer.delay(0.3);
			operation++;
		}
		if(operation == 15)
		{
			ClawStandby();
			operation++;
		}
		if(operation == 16)
		{
			operation += pidTurnExact(-30);
		}
		if(operation == 17)
		{
			operation += driveForward(1900, -30, false, backLeftDrive);
		}
		if(operation == 18)
		{
			operation += pidTurnExact(0.0);
		}
		if(operation == 19)
		{
			operation += elevatorPosition(52000);
		}
		if(operation == 20)
		{
			operation += driveForward(300, 0.0, true, backLeftDrive);
		}
		if(operation == 21)
		{
			ClawOutput();
			Timer.delay(0.3);
			operation++;
		}
		if(operation == 22)
		{
			ClawStandby();
			operation++;
		}
	}

	private void right_LeftScaleLeftSwitch()
	{
		if(operation == 0)
		{
			operation += driveForward(5200, 0.0, true, frontRightDrive);
		}
		if(operation == 1)
		{
			operation += driveLeft(7700, 0.0, backLeftDrive);
		}
		if(operation == 2)
		{
			operation += driveForward(2000, 0.0, true, frontRightDrive);
		}
		if(operation == 3)
		{
			operation += pidTurnExact(60.0);
		}
		if(operation == 4)
		{
			operation += elevatorPosition(35000);
		}
		if(operation == 5)
		{
			ClawRotateUp();
			Timer.delay(0.075);
			operation++;
		}
		if(operation == 6)
		{
			ClawStop();
			Timer.delay(0.1);
			operation++;
		}
		if(operation == 7)
		{
			ClawOutput();
			Timer.delay(0.5);
			operation++;
		}
		if(operation == 8)
		{
			ClawStandby();
			operation++;
		}
		if(operation == 9)
		{
			operation += elevatorPosition(1000);
		}
		if(operation == 10)
		{
			operation += pidTurnExact(150);
		}
		if(operation == 11)
		{
			ClawRotateUp();
			Timer.delay(0.5);
			operation++;
		}
		if(operation == 12)
		{
			ClawStop();
			ClawOpen();
			ClawIntake();
			operation++;
		}
		if(operation == 13)
		{
			operation += driveForward(2100, 150.0, false, frontRightDrive);
		}
		if(operation == 14)
		{
			ClawClose();
			Timer.delay(1.0);
			operation++;
		}
		if(operation == 15)
		{
			ClawStandby();
			operation++;
		}
		if(operation == 16)
		{
			operation += elevatorPosition(30000);
		}
		if(operation == 17)
		{
			operation += driveForward(1000, 150.0, false, frontRightDrive);
		}
		if(operation == 18)
		{
			ClawOpen();
			operation++;
		}
	}

	private void right_RightScale()
	{
		if(operation == 0)
		{
			operation += driveForward(8000, 0.0, true, backLeftDrive);
		}
		if(operation == 1)
		{
			operation += pidTurnExact(-60.0);
		}
		if(operation == 2)
		{
			operation += elevatorPosition(35000);
		}
		if(operation == 3)
		{
			ClawRotateUp();
			Timer.delay(0.075);
			operation++;
		}
		if(operation == 4)
		{
			ClawStop();
			Timer.delay(0.1);
			operation++;
		}
		if(operation == 5)
		{
			ClawOutput();
			Timer.delay(0.5);
			operation++;
		}
		if(operation == 6)
		{
			ClawStandby();
			operation++;
		}
		if(operation == 7)
		{
			operation += elevatorPosition(1000);
		}
	}

	private void right_RightScaleRightSwitch()
	{
		if(operation == 0)
		{
			operation += driveForward(6900, 0.0, true, backLeftDrive);
		}
		if(operation == 1)
		{
			operation += pidTurnExact(-60.0);
		}
		if(operation == 2)
		{
			operation += elevatorPosition(35000);
		}
		if(operation == 3)
		{
			ClawRotateUp();
			Timer.delay(0.075);
			operation++;
		}
		if(operation == 4)
		{
			ClawStop();
			Timer.delay(0.1);
			operation++;
		}
		if(operation == 5)
		{
			ClawOutput();
			Timer.delay(0.5);
			operation++;
		}
		if(operation == 6)
		{
			ClawStandby();
			operation++;
		}
		if(operation == 4)
		{
			operation += elevatorPosition(1000);
		}
		if(operation == 5)
		{
			operation += pidTurnExact(-150);
		}
		if(operation == 6)
		{
			ClawRotateUp();
			Timer.delay(0.5);
			operation++;
		}
		if(operation == 7)
		{
			ClawStop();
			ClawOpen();
			ClawIntake();
			operation++;
		}
		if(operation == 8)
		{
			operation += driveForward(2100, -150.0, false, backLeftDrive);
		}
		if(operation == 9)
		{
			ClawClose();
			Timer.delay(1.0);
			operation++;
		}
		if(operation == 10)
		{
			ClawStandby();
			operation++;
		}
		if(operation == 11)
		{
			operation += elevatorPosition(30000);
		}
		if(operation == 12)
		{
			operation += driveForward(1000, -150.0, false, backLeftDrive);
		}
		if(operation == 13)
		{
			ClawOpen();
			operation++;
		}
	}

	private void right_TwoRightScale()
	{
		if(operation == 0)
		{
			operation += driveForward(7500, 0.0, true, backLeftDrive);
		}
		if(operation == 1)
		{
			operation += pidTurnExact(-60.0);
		}
		if(operation == 2)
		{
			operation += elevatorPosition(35000);
		}
		if(operation == 3)
		{
			ClawRotateUp();
			Timer.delay(0.075);
			operation++;
		}
		if(operation == 4)
		{
			ClawStop();
			Timer.delay(0.1);
			operation++;
		}
		if(operation == 5)
		{
			ClawOutput();
			Timer.delay(0.5);
			operation++;
		}
		if(operation == 6)
		{
			ClawStandby();
			operation++;
		}
		if(operation == 7)
		{
			operation += elevatorPosition(1000);
		}
		if(operation == 8)
		{
			operation += pidTurnExact(-150);
		}
		if(operation == 9)
		{
			ClawRotateUp();
			Timer.delay(0.5);
			operation++;
		}
		if(operation == 10)
		{
			ClawStop();
			ClawOpen();
			ClawIntake();
			operation++;
		}
		if(operation == 11)
		{
			operation += driveForward(2100, -150.0, false, backLeftDrive);
		}
		if(operation == 10)
		{
			ClawClose();
			Timer.delay(0.3);
			operation++;
		}
		if(operation == 11)
		{
			ClawStandby();
			operation++;
		}
		if(operation == 12)
		{
			operation += pidTurnExact(30);
		}
		if(operation == 13)
		{
			operation += driveForward(2000, 30, false, backLeftDrive);
		}
		if(operation == 14)
		{
			operation += pidTurnExact(0.0);
		}
		if(operation == 15)
		{
			operation += elevatorPosition(52000);
		}
		if(operation == 16)
		{
			operation += driveForward(300, 0.0, true, backLeftDrive);
		}
		if(operation == 17)
		{
			ClawOutput();
			Timer.delay(0.5);
		}
		if(operation == 18)
		{
			ClawStandby();
			operation++;
		}
	}
}
