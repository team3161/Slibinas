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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot implements PIDOutput
{ 
	//This is declaring the Motors with the corresponding Controllers
	private WPI_TalonSRX frontLeftDrive = new WPI_TalonSRX(0);
	private WPI_TalonSRX frontRightDrive = new WPI_TalonSRX(1);
	private WPI_TalonSRX backLeftDrive = new WPI_TalonSRX(2);
	private WPI_TalonSRX backRightDrive = new WPI_TalonSRX(3);
	private VictorSP IntakeL = new VictorSP (3);
	private VictorSP IntakeR = new VictorSP (2);
	private VictorSP pivot = new VictorSP (0);
	private WPI_TalonSRX leftElevator = new WPI_TalonSRX (4);
	private WPI_TalonSRX rightElevator = new WPI_TalonSRX (5);

	//Declaring the way the robot will drive - RoboDrive class
	private MecanumDrive drivetrain;

	//Declaring the AHRS class to get gyro headings
	private AHRS ahrs;
	private double angle;

	//This is declaring both Controllers 
	private LogitechDualAction driverPad = new LogitechDualAction(0);
	private LogitechDualAction operatorPad = new LogitechDualAction(1);

	//For drive train PID - face buttons
	PIDController turnController;
	double rotate;
	double P = 0.02;
	double I = 0.00;
	double D = 0.06;
	float kToleranceDegrees = 2;
	boolean rotateToAngle;
	double currentRotationRate;

	//For drive train - wheel rotations
	//Front Left
	PIDController FLController;
	double P0 = 0.02;
	double I0 = 0.00;
	double D0 = 0.06;
	double FLSpeed;
	//Front Right
	PIDController FRController;
	double P1 = 0.02;
	double I1 = 0.00;
	double D1 = 0.06;
	double FRSpeed;
	//Back Left
	PIDController BLController;
	double P2 = 0.02;
	double I2 = 0.00;
	double D2 = 0.06;
	double BLSpeed;
	//Back Right
	PIDController BRController;
	double P3 = 0.02;
	double I3 = 0.00;
	double D3 = 0.06;
	double BRSpeed;

	//For elevator PID
	PIDController elevatorController;
	boolean rotatetoHeight;

	//Configures all the Joysticks and Buttons for both controllers
	double leftStickX;
	double leftStickY;	 
	double rightStickX;
	double rightStickY;
	double leftStickY_Operator;
	boolean getButtonSTART;
	boolean getButtonY;
	boolean getButtonX;
	boolean getButtonA;
	boolean getButtonB;
	boolean getButtonRT_Operator;
	boolean getButtonLB_Operator;
	boolean getButtonLT_Operator;
	double rightStickY_Operator;
	boolean test;

	//need to set variables to use PCM
	private Compressor air = new Compressor(0);
	private boolean pressureSwitch;

	//Declaring Buttons for the pistons
	private DoubleSolenoid claw = new DoubleSolenoid (1,0);

	//Declaring a string to get the switch/scale positioning for each alliance
	String gameData;

	//Declaring a timer for autonomous timings
	Timer autoTimer = new Timer();

	//Declaring positions for starting autonomous
	boolean MIDDLE = true, LEFT = false, RIGHT = false;
	double ticks = 0;

	//A counter for running the claw's intake after the claw closes
	private int c = 0;

	private int operation = 0;

	public void robotInit() 
	{
		frontLeftDrive.setInverted(false);
		frontRightDrive.setInverted(false);
		backRightDrive.setInverted(false);
		backLeftDrive.setInverted(false);

		//gyro readings for kmxp
		//ahrs = new AHRS(SPI.Port.kMXP);

		//Reads in gyro readings from I2C connections
		ahrs = new AHRS (I2C.Port.kOnboard);

		//Initiate the RoboDrive class, so that drive train variable can be used with the talons - driving controller
		drivetrain = new MecanumDrive(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive);

		//Resetting the gyro reading for the rest of run time
		ahrs.reset();

		//Configuring settings for wheel encoders
		frontLeftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		frontLeftDrive.set(ControlMode.Position, 0);
		frontRightDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		frontRightDrive.set(ControlMode.Position, 0);
		backLeftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		backLeftDrive.set(ControlMode.Position, 0);
		backRightDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		backRightDrive.set(ControlMode.Position, 0);

		//Executes PID calculations for gyro face buttons
		turnController = new PIDController(P, I, D, ahrs, this::pidWrite);
		turnController.setInputRange(-180.0f,  180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);

		//Executes PID calculations for wheel rotations - Front Left
		FLController = new PIDController(P0, I0, D0, new TalonSrxPIDSource(frontLeftDrive), this::frontLeftPidWrite);
		FLController.setInputRange(-100000, 100000);
		FLController.setOutputRange(-1.0, 1.0);
		FLController.setAbsoluteTolerance(kToleranceDegrees);

		//Executes PID calculations for wheel rotations - Front Right
		FRController = new PIDController(P1, I1, D1, new TalonSrxPIDSource(frontRightDrive), this::frontRightPidWrite);
		FRController.setInputRange(-100000, 100000);
		FRController.setOutputRange(-1.0, 1.0);
		FRController.setAbsoluteTolerance(kToleranceDegrees);

		//Executes PID calculations for wheel rotations - Back Left
		BLController = new PIDController(P2, I2, D2, new TalonSrxPIDSource(backLeftDrive), this::backLeftPidWrite);
		BLController.setInputRange(-100000, 100000);
		BLController.setOutputRange(-1.0, 1.0);
		BLController.setAbsoluteTolerance(kToleranceDegrees);

		//Executes PID calculations for wheel rotations - Back Right
		BRController = new PIDController(P3, I3, D3, new TalonSrxPIDSource(backRightDrive), this::backRightPidWrite);
		BRController.setInputRange(-100000, 100000);
		BRController.setOutputRange(-1.0, 1.0);
		BRController.setAbsoluteTolerance(kToleranceDegrees);


		//Executes PID calculations for wheel rotations - Y Direction
		/*
		FRController = new PIDController(Py, Iy, Dy, ahrs, this);
		FRController.setInputRange(-100000, 100000);
		FRController.setOutputRange(-1.0,  1.0);
		FRController.setAbsoluteTolerance(kToleranceDegrees);
		 */


		//WHEN LEFT SIDE WHEELS ROTATE FORWARDS OVER THE TOP, ENCODERS APPROACH -INFINITY
		//WHEN RIGHT SIDE WHEELS ROTATE FORWARDS OVER THE TOP, ENCODERS APPROACH +INFINITY

		//Executes PID calculations for elevator
		//elevatorController = new PIDController(P, I, D, (PIDSource) leftElevator, this);

		//The robot's claw is set to closed because it will be holding a cube when turned on
		ClawClose();

		//with proper wiring, calibrates the talons using polarity inversions
		driverPad.setMode(LogitechControl.LEFT_STICK, LogitechAxis.X, new SquaredJoystickMode());
		driverPad.setMode(LogitechControl.LEFT_STICK, LogitechAxis.Y, new InvertedJoystickMode().andThen(new SquaredJoystickMode()));
		driverPad.setMode(LogitechControl.RIGHT_STICK, LogitechAxis.X, new SquaredJoystickMode());

		//Configuring settings for the elevator encoders

		//Configuring elevator encoders
		//leftElevator.set(ControlMode.Position, 0);;
		//leftElevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
	}

	public void autonomousInit() 
	{
		ahrs.reset();
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		resetEncoders();
		autoTimer.start();
	}

	public void autonomousPeriodic()
	{
		driverPad.setMode(LogitechControl.LEFT_STICK, LogitechAxis.X, new SquaredJoystickMode());
		driverPad.setMode(LogitechControl.LEFT_STICK, LogitechAxis.Y, new InvertedJoystickMode().andThen(new SquaredJoystickMode()));
		driverPad.setMode(LogitechControl.RIGHT_STICK, LogitechAxis.X, new SquaredJoystickMode());

		//get air for pneumatics
		/*pressureSwitch = air.getPressureSwitchValue();
		if (!pressureSwitch) 
		{
			air.setClosedLoopControl(true);
		}

		 */		
		FLController.setSetpoint(1000.0f);
		FLController.setEnabled(true);
		frontLeftDrive.set(-FLSpeed);
		/*
		if(MIDDLE && gameData.charAt(0) == 'L')
		{
			if(operation == 0)
			{
				driveForward(375, 0.3, 'f');
			}
			if(operation == 1)
			{
				operation += driveLeft(200, 0.3, 'f');
			}
			if(operation == 2)
			{
				operation += driveForward(175, 0.3, 'f');
			}
			if(operation == 3)
			{
				//ClawOutput();
				operation++;
			}
			if(operation == 4)
			{
				operation += driveLeft(150, 0.3, 'f');
			}
			if(operation == 5)
			{
				operation += driveForward(100, 0.3, 'f');
			}

			//Raise elevator to proper height using VP encoders
		}
		 */


		//stop taking air when pneumatics reaches 120 psi
		/*if (pressureSwitch) 
		{
			air.setClosedLoopControl(false);
		}
		 */
		showDisplay();
	}

	public void teleopPeriodic() 
	{
		showDisplay();
		
		leftStickX = driverPad.getValue(LogitechDualAction.LogitechControl.LEFT_STICK, LogitechDualAction.LogitechAxis.X);
		leftStickY = driverPad.getValue(LogitechDualAction.LogitechControl.LEFT_STICK, LogitechDualAction.LogitechAxis.Y);
		rightStickX = driverPad.getValue(LogitechDualAction.LogitechControl.RIGHT_STICK, LogitechDualAction.LogitechAxis.X);
		getButtonSTART = driverPad.getButton(LogitechDualAction.LogitechButton.START);
		getButtonY = driverPad.getButton(LogitechDualAction.LogitechButton.Y);
		getButtonX = driverPad.getButton(LogitechDualAction.LogitechButton.X);
		getButtonA = driverPad.getButton(LogitechDualAction.LogitechButton.A);
		getButtonRT_Operator = operatorPad.getButton(LogitechDualAction.LogitechButton.RIGHT_TRIGGER);
		getButtonB = driverPad.getButton(LogitechDualAction.LogitechButton.B);
		getButtonLB_Operator = operatorPad.getButton(LogitechDualAction.LogitechButton.LEFT_BUMPER);
		getButtonLT_Operator = operatorPad.getButton(LogitechDualAction.LogitechButton.LEFT_TRIGGER);
		leftStickY_Operator = operatorPad.getValue(LogitechDualAction.LogitechControl.LEFT_STICK, LogitechDualAction.LogitechAxis.Y);
		rightStickY_Operator = operatorPad.getValue(LogitechDualAction.LogitechControl.RIGHT_STICK, LogitechDualAction.LogitechAxis.Y );
		test = driverPad.getButton(LogitechDualAction.LogitechButton.RIGHT_TRIGGER);

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
		if (getButtonY) 
		{
			currentRotationRate = forwardPID();
			rotateToAngle = true;
		}
		else if (getButtonB) 
		{
			currentRotationRate = rightPID();
			rotateToAngle = true;
		}
		else if (getButtonA) 
		{
			currentRotationRate = backwardPID();
			rotateToAngle = true;
		}
		else if (getButtonX) 
		{
			currentRotationRate = leftPID();
			rotateToAngle = true;
		}

		if(!rotateToAngle)
		{
			turnController.disable();
			currentRotationRate = rightStickX;
		}
		
		if(test)
		{
			/*
			FLController.setSetpoint(10000);
			FLController.enable();
			frontLeftDrive.set(-FLSpeed);
			*/
			frontLeftDrive.set(0.5);
		}

		//Calls upon the mecanumDrive_Cartesian method that sends specific power to the talons
		drivetrain.driveCartesian (leftStickX, leftStickY, currentRotationRate * 0.5, -angle);

		//Setting speeds for the claw's motors to intake or shoot out the cube
		//Left Bumper intakes, Left Trigger spits out cube
		if(getButtonLT_Operator)
		{
			ClawOutput();
		}
		else {
			ClawStandby();
		}

		//Setting positions for the pistons
		//Pressing Right Trigger opens the claw, claw is closed at default
		if(getButtonRT_Operator)
		{
			c = 0;
			ClawOpen();
			ClawIntake();
		}else
		{
			c++;
			if(c <= 60)
			{
				ClawIntake();
			}
			if(c == 100000)
			{
				c = 0;
			}
			ClawClose();
		}

		//Move the elevator up or down
		if (leftStickY_Operator > 0.25)
		{
			ElevatorDown();
		}else if (leftStickY_Operator < -0.25)
		{
			ElevatorUp();
		}
		else 
		{
			//elevatorController.setSetpoint(leftElevator.getSelectedSensorPosition(0));
			//elevatorController.enable();
			//leftStickY_Operator = rotateToAngleRate;
		}

		//Stop taking air when pneumatics reaches 120 psi
		if (pressureSwitch == true) 
		{
			air.setClosedLoopControl(false);
		}

		//Right stick on operator controller lowers claw based on input - otherwise it holds
		if(rightStickY_Operator > 0.25)
		{
			ClawRotateUp();
		}else if(rightStickY_Operator < -0.25) 
		{
			ClawRotateDown();
		}else
		{
			ClawHold();
		}
	}

	public void pidWrite(double output) 
	{
		rotate = output;
	}

	public void frontLeftPidWrite(double speed0)
	{
		FLSpeed = speed0;
		SmartDashboard.putNumber("KTY", speed0);
	}

	public void frontRightPidWrite(double speed1)
	{
		FRSpeed = speed1;
	}

	public void backLeftPidWrite(double speed2)
	{
		BLSpeed = speed2;
	}

	public void backRightPidWrite(double speed3)
	{
		BRSpeed = speed3;
	}

	//Preset method that pushes out "right stick X rotation" with PID - backward
	private double forwardPID() 
	{
		turnController.setSetpoint(0.0f);
		turnController.enable();
		return rotate;
	}

	//Preset method that pushes out "right stick X rotation" with PID - forward
	private double backwardPID() 
	{
		turnController.setSetpoint(180.0f);
		turnController.enable();
		return rotate;
	}

	//Preset method that pushes out "right stick X rotation" with PID - right
	private double rightPID() 
	{
		turnController.setSetpoint(90.0f);
		turnController.enable();
		return rotate;
	}

	//Preset method that pushes out "right stick X rotation" with PID - left
	private double leftPID() 
	{
		turnController.setSetpoint(-90.0f);
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
	//Motors pulling cube in lightly to prevent losing the cube 
	private void ClawStandby()
	{
		IntakeL.set(0.1);
		IntakeR.set(-0.1);
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
		leftElevator.set(-1);
		rightElevator.set(1);
	}
	//Elevator Down
	private void ElevatorDown()
	{
		leftElevator.set(0.35);
		rightElevator.set(-0.35);
	}

	//Elevator Stop
	private void ElevatorHold()
	{
		//leftElevator.set(ControlMode.Position, height);
		rightElevator.set(0.0);
		leftElevator.set(0.0);
	}

	//Claw Rotate Up
	private void ClawRotateUp()
	{
		pivot.set(0.6);
	}

	//Claw Rotate Down
	private void ClawRotateDown()
	{
		pivot.set(-0.6);
	}

	//Claw motors are stopped
	private void ClawHold()
	{
		pivot.set(0.0);
	}

	//Driving forward using encoders
	private int driveForward(double ticks, char orientation)
	{

		double currentRotationRate = 0;
		if(orientation == 'f')
		{
			currentRotationRate = forwardPID();
		}else if(orientation == 'l')
		{
			currentRotationRate = leftPID();
		}
		else if(orientation == 'r')
		{
			currentRotationRate = rightPID();
		}else if(orientation == 'b')
		{
			currentRotationRate = backwardPID();
		}

		angle = ahrs.getYaw();

		drivetrain.driveCartesian(0, 0, 0, -angle);

		return 0;

	}

	//Driving left using encoders
	private int driveLeft(double ticks, char orientation)
	{

		double currentRotationRate = 0;
		if(orientation == 'f')
		{
			currentRotationRate = forwardPID();
		}else if(orientation == 'l')
		{
			currentRotationRate = leftPID();
		}
		else if(orientation == 'r')
		{
			currentRotationRate = rightPID();
		}else if(orientation == 'b')
		{
			currentRotationRate = backwardPID();
		}

		angle = ahrs.getYaw();

		drivetrain.driveCartesian(0, 0, currentRotationRate, -angle);

		return 0;
	}

	//Driving right using encoders
	private int driveRight(double ticks, char orientation)
	{


		double currentRotationRate = 0;
		if(orientation == 'f')
		{
			currentRotationRate = forwardPID();
		}else if(orientation == 'l')
		{
			currentRotationRate = leftPID();
		}
		else if(orientation == 'r')
		{
			currentRotationRate = rightPID();
		}else if(orientation == 'b')
		{
			currentRotationRate = backwardPID();
		}

		angle = ahrs.getYaw();

		drivetrain.driveCartesian(0, 0, currentRotationRate, -angle);

		return 0;
	}

	//Driving forward using encoders
	private int driveBackward(double ticks, char orientation)
	{


		double currentRotationRate = 0;
		if(orientation == 'f')
		{
			currentRotationRate = forwardPID();
		}else if(orientation == 'l')
		{
			currentRotationRate = leftPID();
		}
		else if(orientation == 'r')
		{
			currentRotationRate = rightPID();
		}else if(orientation == 'b')
		{

			angle = ahrs.getYaw();	

			drivetrain.driveCartesian(0, 0, currentRotationRate, -angle);}

		return 0;
	}

	//Returns the average wheel rotations from all four wheels
	private double wheelRotations()
	{
		return (frontLeftDrive.getSelectedSensorPosition(0) + frontRightDrive.getSelectedSensorPosition(1) + backLeftDrive.getSelectedSensorPosition(2) +  backRightDrive.getSelectedSensorPosition(3)) / 4;
	}

	private void resetEncoders()
	{
		frontLeftDrive.setSelectedSensorPosition(0, 0, 10);
		frontRightDrive.setSelectedSensorPosition(0, 0, 10);
		backLeftDrive.setSelectedSensorPosition(0, 0, 10);
		backRightDrive.setSelectedSensorPosition(0, 0, 10);
	}

	private void showDisplay()
	{
		SmartDashboard.putNumber("Front Left Encoder:", frontLeftDrive.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Front Right Encoder:", frontRightDrive.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Back Left Encoder:", backLeftDrive.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Back Right Encoder:", backRightDrive.getSelectedSensorPosition(0));
	}
}
