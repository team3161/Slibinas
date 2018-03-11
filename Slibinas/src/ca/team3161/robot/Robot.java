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
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot
{ 
	//This is declaring the motors with the corresponding Controllers
	private WPI_TalonSRX frontLeftDrive = new WPI_TalonSRX(0);
	private WPI_TalonSRX frontRightDrive = new WPI_TalonSRX(1);
	private WPI_TalonSRX backLeftDrive = new WPI_TalonSRX(2);
	private WPI_TalonSRX backRightDrive = new WPI_TalonSRX(3);
	private VictorSP IntakeL = new VictorSP (3);
	private VictorSP IntakeR = new VictorSP (2);
	private VictorSP pivot = new VictorSP (1);
	private WPI_TalonSRX leftElevator = new WPI_TalonSRX (4);
	private VictorSP leftElevatorSlave = new VictorSP(0);
	private WPI_TalonSRX rightElevator = new WPI_TalonSRX (6);

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
	double Pg = 0.02;
	double Ig = 0.00;
	double Dg = 0.06;
	float kToleranceDegrees = 2;
	boolean rotateToAngle;
	double currentRotationRate;

	//For drive train - wheel rotations
	//Front Left
	float kToleranceRotations = 100;
	PIDController FLController;
	double P0 = 0.0004;
	double I0 = 0.000001;
	double D0 = 0.0001;
	double FLSpeed;
	//Front Right
	PIDController FRController;
	double P1 = 0.0004;
	double I1 = 0.000001;
	double D1 = 0.0001;
	double FRSpeed;
	//Back Left
	PIDController BLController;
	double P2 = 0.0004;
	double I2 = 0.000001;
	double D2 = 0.0001;
	double BLSpeed;
	//Back Right
	PIDController BRController;
	double P3 = 0.0004;
	double I3 = 0.000001;
	double D3 = 0.0001;
	double BRSpeed;
	//A tolerance value that wheel rotations must reach between
	private double tolerance = 200;

	//For elevator PID
	PIDController EController;
	double Pe = 0.0;
	double Ie = 0.0;
	double De = 0.0;
	double ESpeed;
	//A value assigned to top, bottom, switch and scale height
	double TOP = 0.0;
	double BOTTOM = 0.0;
	double SWITCH = 0.0;
	double SCALE = 0.0;

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
	boolean getButtonRT_Operator;
	boolean getButtonLB_Operator;
	boolean getButtonLT_Operator;

	//need to set variables to use PCM
	private Compressor air = new Compressor(0);
	private boolean pressureSwitch;

	//Declaring Buttons for the pistons
	private DoubleSolenoid claw = new DoubleSolenoid (1,0);

	//Declaring a string to get the switch/scale positioning for each alliance
	String gameData;

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
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);		

		//Executes PID calculations for wheel rotations - Front Left
		FLController = new PIDController(P0, I0, D0, new TalonSrxPIDSource(frontLeftDrive, 0), this::frontLeftPidWrite);
		FLController.setInputRange(-100000, 100000);
		FLController.setOutputRange(-0.75, 0.75);
		FLController.setAbsoluteTolerance(kToleranceRotations);
		FLController.setContinuous(false);


		//Executes PID calculations for wheel rotations - Front Right
		FRController = new PIDController(P1, I1, D1, new TalonSrxPIDSource(frontRightDrive, 0), this::frontRightPidWrite);
		FRController.setInputRange(-100000, 100000);
		FRController.setOutputRange(-0.75, 0.75);
		FRController.setAbsoluteTolerance(kToleranceRotations);
		FRController.setContinuous(false);

		//Executes PID calculations for wheel rotations - Back Left
		BLController = new PIDController(P2, I2, D2, new TalonSrxPIDSource(backLeftDrive, 0), this::backLeftPidWrite);
		BLController.setInputRange(-100000, 100000);
		BLController.setOutputRange(-0.75, 0.75);
		BLController.setAbsoluteTolerance(kToleranceRotations);
		BLController.setContinuous(false);

		//Executes PID calculations for wheel rotations - Back Right
		BRController = new PIDController(P3, I3, D3, new TalonSrxPIDSource(backRightDrive, 0), this::backRightPidWrite);
		BRController.setInputRange(-100000, 100000);
		BRController.setOutputRange(-0.75, 0.75);
		BRController.setAbsoluteTolerance(kToleranceRotations);
		BLController.setContinuous(false);
		
		//Executes PID calculations for elevator rotations
		EController = new PIDController(Pe, Ie, De, new TalonSrxPIDSource(rightElevator, 0), this::ElevatorPidWrite);
		EController.setInputRange(-100000, 100000);
		EController.setOutputRange(-0.7, 1.0);
		EController.setAbsoluteTolerance(kToleranceRotations);
		EController.setContinuous(false);
		
		//WHEN LEFT SIDE WHEELS ROTATE FORWARDS OVER THE TOP, ENCODERS APPROACH -INFINITY
		//WHEN RIGHT SIDE WHEELS ROTATE FORWARDS OVER THE TOP, ENCODERS APPROACH +INFINITY

		//The robot's claw is set to closed because it will be holding a cube when turned on
		ClawClose();

		//with proper wiring, calibrates the talons using polarity inversions
		driverPad.setMode(LogitechControl.LEFT_STICK, LogitechAxis.X, new SquaredJoystickMode());
		driverPad.setMode(LogitechControl.LEFT_STICK, LogitechAxis.Y, new InvertedJoystickMode().andThen(new SquaredJoystickMode()));
		driverPad.setMode(LogitechControl.RIGHT_STICK, LogitechAxis.X, new SquaredJoystickMode());

		//Configuring settings for the elevator encoders

	}

	public void autonomousInit() 
	{
		ahrs.reset();
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		resetAllEncoders();
		FLController.enable();
		FRController.enable();
		BLController.enable();
		BRController.enable();
		EController.enable();
	}

	public void autonomousPeriodic()
	{
		//get air for pneumatics
		/*pressureSwitch = air.getPressureSwitchValue();
		if (!pressureSwitch) 
		{
			air.setClosedLoopControl(true);
		}

		 */	

		driveRight(5000);

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

	public void teleopInit()
	{
		ahrs.reset();
		resetAllEncoders();
		FLController.disable();
		FRController.disable();
		BLController.disable();
		BRController.disable();
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

		//Calls upon the mecanumDrive_Cartesian method that sends specific power to the talons
		drivetrain.driveCartesian (leftStickX, leftStickY, currentRotationRate * 0.5, -angle);

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
			ElevatorHold();
		}

		//Right stick on operator controller raises/lowers claw based on input - otherwise it holds
		if(rightStickY_Operator > 0.25)
		{
			ClawRotateUp();
		}else if(rightStickY_Operator < -0.25) 
		{
			ClawRotateDown();
		}else
		{
			ClawStop();
		}

		//Stop taking air when pneumatics reaches 120 psi
		if (pressureSwitch == true) 
		{
			air.setClosedLoopControl(false);
		}
	}

	//Receives gyro PID values
	public void gyroPidWrite(double output) 
	{
		rotate = output;
	}

	//Receives front left motor PID values 
	public void frontLeftPidWrite(double speed0)
	{
		FLSpeed = -speed0;
	}

	//Receives front right motor PID values 
	public void frontRightPidWrite(double speed1)
	{
		FRSpeed = -speed1;
	}

	//Receives back left motor PID values 
	public void backLeftPidWrite(double speed2)
	{
		BLSpeed = -speed2;
	}

	//Receives back right motor PID values 
	public void backRightPidWrite(double speed3)
	{
		BRSpeed = -speed3;
	}

	//Receives right elevator PID values
	public void ElevatorPidWrite(double speed4)
	{
		ESpeed = -speed4;
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
		leftElevator.set(-0.7);
		leftElevatorSlave.set(-0.7);
		rightElevator.set(-0.7);
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

	//Claw motors are stopped
	private void ClawStop()
	{
		pivot.set(0.0);
	}

	private int pidTurn(char direction)
	{
		double turnSpeed;
		double angle;
		//Setting a direction for the robot to face
		switch(direction)
		{
		case 'f': turnSpeed = forwardPID();
		angle = 0.0;
		break;
		case 'l': turnSpeed = leftPID();
		angle = -90.0;
		break;
		case 'r': turnSpeed = rightPID();
		angle = 90.0;
		break;
		case 'b': turnSpeed = backwardPID();
		angle = 180.0;
		break;
		default: turnSpeed = 0.0;
		angle = 0.0;
		}

		drivetrain.driveCartesian(0.0, 0.0, turnSpeed, 0.0);

		if(ahrs.getYaw() >= angle - kToleranceDegrees && ahrs.getYaw() <= angle + kToleranceDegrees)
		{
			return 1;
		}else
		{
			return 0;
		}
	}

	//Getting elevator to desired height
	private int elevatorPosition(double position)
	{
		EController.setSetpoint(position);
		rightElevator.set(ESpeed);
		leftElevator.set(ESpeed);
		leftElevatorSlave.set(ESpeed);
		
		if(Math.abs(getTicks(rightElevator)) >= position - tolerance && Math.abs(getTicks(rightElevator)) <= position + tolerance)
		{
			return 1;
		}else
		{
			return 0;
		}
	}

	//Driving forward using encoders
	private int driveForward(double ticks)
	{
		//FRONT LEFT
		FLController.setSetpoint(-ticks);
		frontLeftDrive.set(FLSpeed);

		//FRONT RIGHT
		FRController.setSetpoint(ticks);
		frontRightDrive.set(FRSpeed);

		//BACK LEFT
		BLController.setSetpoint(-ticks);
		backLeftDrive.set(BLSpeed);

		//BACK RIGHT
		BRController.setSetpoint(ticks);
		backRightDrive.set(BRSpeed);

		if(Math.abs(getTicks(frontLeftDrive)) >= ticks - tolerance && Math.abs(getTicks(frontLeftDrive)) <= ticks + tolerance)
		{
			if(Math.abs(getTicks(frontRightDrive)) >= ticks - tolerance && Math.abs(getTicks(frontRightDrive)) <= ticks + tolerance)
			{
				if(Math.abs(getTicks(backLeftDrive)) >= ticks - tolerance && Math.abs(getTicks(backLeftDrive)) <= ticks + tolerance)
				{
					if(Math.abs(getTicks(backRightDrive)) > ticks - tolerance && Math.abs(getTicks(backRightDrive)) <= ticks + tolerance)
					{
						return 1;
					}
				}
			}
		}
		return 0;
	}

	//Driving left using encoders
	private int driveLeft(double ticks)
	{
		//FRONT LEFT
		FLController.setSetpoint(ticks);
		frontLeftDrive.set(FLSpeed);

		//FRONT RIGHT
		FRController.setSetpoint(ticks);
		frontRightDrive.set(FRSpeed);

		//BACK LEFT
		BLController.setSetpoint(-ticks);
		backLeftDrive.set(BLSpeed);

		//BACK RIGHT
		BRController.setSetpoint(-ticks);
		backRightDrive.set(BRSpeed);

		if(Math.abs(getTicks(frontLeftDrive)) >= ticks - tolerance && Math.abs(getTicks(frontLeftDrive)) <= ticks + tolerance)
		{
			if(Math.abs(getTicks(frontRightDrive)) >= ticks - tolerance && Math.abs(getTicks(frontRightDrive)) <= ticks + tolerance)
			{
				if(Math.abs(getTicks(backLeftDrive)) >= ticks - tolerance && Math.abs(getTicks(backLeftDrive)) <= ticks + tolerance)
				{
					if(Math.abs(getTicks(backRightDrive)) >= ticks - tolerance && Math.abs(getTicks(backRightDrive)) <= ticks + tolerance)
					{
						return 1;
					}
				}
			}
		}
		return 0;
	}

	//Driving right using encoders
	private int driveRight(double ticks)
	{
		//FRONT LEFT
		FLController.setSetpoint(-ticks);
		frontLeftDrive.set(FLSpeed);

		//FRONT RIGHT
		FRController.setSetpoint(-ticks);
		frontRightDrive.set(FRSpeed);

		//BACK LEFT
		BLController.setSetpoint(ticks);
		backLeftDrive.set(BLSpeed);

		//BACK RIGHT
		BRController.setSetpoint(ticks);
		backRightDrive.set(BRSpeed);

		if(Math.abs(getTicks(frontLeftDrive)) >= ticks - tolerance && Math.abs(getTicks(frontLeftDrive)) <= ticks + tolerance)
		{
			if(Math.abs(getTicks(frontRightDrive)) >= ticks - tolerance && Math.abs(getTicks(frontRightDrive)) <= ticks + tolerance)
			{
				if(Math.abs(getTicks(backLeftDrive)) >= ticks - tolerance && Math.abs(getTicks(backLeftDrive)) <= ticks + tolerance)
				{
					if(Math.abs(getTicks(backRightDrive)) > ticks - tolerance && Math.abs(getTicks(backRightDrive)) <= ticks + tolerance)
					{
						return 1;
					}
				}
			}
		}
		return 0;
	}

	//Driving backward using encoders
	private int driveBackward(double ticks)
	{
		//FRONT LEFT
		FLController.setSetpoint(ticks);
		frontLeftDrive.set(FLSpeed);

		//FRONT RIGHT
		FRController.setSetpoint(-ticks);
		frontRightDrive.set(FRSpeed);

		//BACK LEFT
		BLController.setSetpoint(ticks);
		backLeftDrive.set(BLSpeed);

		//BACK RIGHT
		BRController.setSetpoint(-ticks);
		backRightDrive.set(BRSpeed);

		if(Math.abs(getTicks(frontLeftDrive)) >= ticks - tolerance && Math.abs(getTicks(frontLeftDrive)) <= ticks + tolerance)
		{
			if(Math.abs(getTicks(frontRightDrive)) >= ticks - tolerance && Math.abs(getTicks(frontRightDrive)) <= ticks + tolerance)
			{
				if(Math.abs(getTicks(backLeftDrive)) >= ticks - tolerance && Math.abs(getTicks(backLeftDrive)) <= ticks + tolerance)
				{
					if(Math.abs(getTicks(backRightDrive)) > ticks - tolerance && Math.abs(getTicks(backRightDrive)) <= ticks + tolerance)
					{
						return 1;
					}
				}
			}
		}
		return 0;
	}

	//Resets all encoders
	private void resetAllEncoders()
	{
		frontLeftDrive.setSelectedSensorPosition(0, 0, 10);
		frontRightDrive.setSelectedSensorPosition(0, 0, 10);
		backLeftDrive.setSelectedSensorPosition(0, 0, 10);
		backRightDrive.setSelectedSensorPosition(0, 0, 10);
		rightElevator.setSelectedSensorPosition(0, 0, 10);
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
		SmartDashboard.putNumber("Gyro:", ahrs.getYaw());
		SmartDashboard.putBoolean("Claw Open:", getButtonRT_Operator);
	}
}
