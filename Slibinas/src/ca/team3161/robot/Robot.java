package ca.team3161.robot;

import java.util.EnumSet;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
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
	double Pe = 0.000001;
	double Ie = 0.0;
	double De = 0.00001;
	double ESpeed;
	//A value assigned to top, bottom, switch and scale height
	double TOP = -51000;
	double BOTTOM = -5000;
	double SWITCH = 0.0;

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
	private double ticks = 0;
	
	//Declaring variables to be used in automous
	private double avgSpeed;

	//A counter for running the claw's intake after the claw closes
	private int c = 0;

	//A counter that controls operations in autonomous
	private int operation;

	//objects to help select specific auto paths from the smartdashboard
	private SendableChooser<AutoMode> autoModeChooser;
	private AutoMode selectedAutoMode;

	private enum AutoMode		//Enables a variable to have a set of constant values denoted in upper case
	{
		SWITCH, L_SCALE, R_SCALE, L_SCALE_SWITCH, R_SCALE_SWITCH, L_SCALE_SCALE, R_SCALE_SCALE;
	}

	public void robotInit() 
	{
		//Instances of objects to help select specific autos from the smart dashboard
		autoModeChooser = new SendableChooser<>();
		//EnumSet.complementOf(EnumSet.of(AutoMode.SWITCH)).forEach(mode -> autoModeChooser.addObject(mode.toString(), mode));
		//Default Auto Mode  is scoring switch when starting from middle
		autoModeChooser.addDefault("Default (SWITCH):", AutoMode.SWITCH);
		autoModeChooser.addObject("Starting Left (SCALE):", AutoMode.L_SCALE);
		autoModeChooser.addObject("Starting Right (SCALE)", AutoMode.R_SCALE);
		autoModeChooser.addObject("Starting Left (SCALE & SWITCH)", AutoMode.L_SCALE_SWITCH);
		autoModeChooser.addObject("Starting Right (SCALE & SWITCH)", AutoMode.R_SCALE_SWITCH);
		autoModeChooser.addObject("Starting Left (SCALE & SCALE)", AutoMode.L_SCALE_SCALE);
		autoModeChooser.addObject("Starting Right (SCALE & SCALE)", AutoMode.R_SCALE_SCALE);

		SmartDashboard.putData("AutoModeChooser", autoModeChooser);

		frontLeftDrive.setInverted(false);
		frontRightDrive.setInverted(false);
		backRightDrive.setInverted(false);
		backLeftDrive.setInverted(false);

		//gyro readings for kmxp
		//ahrs = new AHRS(SPI.Port.kMXP);

		//Reads in gyro from I2C connections
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
		EController.setOutputRange(-1.0, 1.0);
		EController.setAbsoluteTolerance(600.0);
		EController.setContinuous(false);

		//WHEN LEFT SIDE WHEELS ROTATE FORWARDS OVER THE TOP, ENCODERS APPROACH -INFINITY
		//WHEN RIGHT SIDE WHEELS ROTATE FORWARDS OVER THE TOP, ENCODERS APPROACH +INFINITY

		//The robot's claw is set to closed because it will be holding a cube when turned on
		ClawClose();

		//with proper wiring, calibrates the talons using polarity inversions
		driverPad.setMode(LogitechControl.LEFT_STICK, LogitechAxis.X, new SquaredJoystickMode());
		driverPad.setMode(LogitechControl.LEFT_STICK, LogitechAxis.Y, new InvertedJoystickMode().andThen(new SquaredJoystickMode()));
		driverPad.setMode(LogitechControl.RIGHT_STICK, LogitechAxis.X, new SquaredJoystickMode());

	}

	public void autonomousInit() 
	{
		ahrs.reset();
		//Get scale/switch positions from driver station
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		resetWheelEncoders();
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

		//SWITCH, L_SCALE, R_SCALE, L_SCALESWITCH, R_SCALESWITCH, L_SCALESCALE, R_SCALESCALE
		switch (selectedAutoMode) {
		case SWITCH:
			SmartDashboard.putString("AUTO", "Centre - Scoring Switch!");
			if(gameData.charAt(0) == 'L')
			{
				//MIDDLE - LEFT SWITCH
				if(operation == 0)
				{
					operation += driveForward(1000, true);
				}
				if(operation == 1)
				{
					operation += driveLeft(1400, true);
				}
				if(operation == 2)
				{
					operation += driveForward(2200, true);
				}
				if(operation == 3)
				{
					operation += pidTurnExact(0.0);
				}
				if(operation == 4)
				{
					Timer.delay(0.3);
					ClawOutput();
					Timer.delay(0.5);
					ClawStandby();
					operation++;
				}
			}else
			{
				//MIDDLE - RIGHT SWITCH
				if(operation == 0)
				{
					operation += driveForward(1000, true);
				}
				if(operation == 1)
				{
					operation += driveRight(1200, true);
				}
				if(operation == 2)
				{
					operation += driveForward(2200, true);
				}
				if(operation == 3)
				{
					operation += pidTurnExact(0.0);
				}
				if(operation == 4)
				{
					pidTurnExact(0.0);
					Timer.delay(0.3);
					ClawOutput();
					Timer.delay(0.5);
					ClawStandby();
					operation++;
				}
			}
			break;
		case L_SCALE:
			SmartDashboard.putString("AUTO", "Left - Scoring Scale!");
			break;
		case R_SCALE:
			SmartDashboard.putString("AUTO", "Right - Scoring Scale!");
			break;
		case L_SCALE_SWITCH:
			SmartDashboard.putString("AUTO", "Left - Scoring Scale... Then Switch!");
			break;
		case R_SCALE_SWITCH:
			SmartDashboard.putString("AUTO", "Right - Scoring Scale... Then Switch!");
			break;
		case L_SCALE_SCALE:
			SmartDashboard.putString("AUTO", "Left - Scoring Scale... Twice!");
			break;
		case R_SCALE_SCALE:
			SmartDashboard.putString("AUTO", "Right  - Scoring Scale... Twice!");
			break;
		}

		//STARTING LEFT - LEFT SCALE
		/*
		if(operation == 0)
		{
			operation += driveForward(7000);
		}
		if(operation == 1)
		{
			operation += pidTurnExact(60.0);
		}
		if(operation == 2)
		{
			operation += elevatorPosition(30000);
		}
		if(operation == 3)
		{
			elevatorPositionHold();
			ClawShoot();
			Timer.delay(0.5);
			ClawStandby();
			operation++;
		}
		if(operation == 4)
		{
			operation += elevatorPosition(5000);
		}
		if(operation == 5)
		{
			elevatorPositionHold();
			pidTurnExact(160);
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
		resetWheelEncoders();
		FLController.disable();
		FRController.disable();
		BLController.disable();
		BRController.disable();
		rightElevator.setSelectedSensorPosition(0, 0, 10);
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
		drivetrain.driveCartesian (leftStickX, leftStickY, currentRotationRate * 0.75, -angle);

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
		claw.set(DoubleSolenoid.Value.kForward);	
	} 

	//Open claw
	private void ClawOpen()
	{
		claw.set(DoubleSolenoid.Value.kReverse);
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

	private int pidTurnExact(double angle)
	{
		double currentAngle = ahrs.getYaw();
		turnController.setSetpoint(angle);
		turnController.enable();

		if(ahrs.getYaw() >= angle - kToleranceDegrees && ahrs.getYaw() <= angle + kToleranceDegrees)
		{
			turnController.disable();
			return 1;
		}else
		{
			drivetrain.driveCartesian(0.0, 0.0, rotate * 0.75, currentAngle);
			return 0;
		}
	}

	//Getting elevator to desired height
	private int elevatorPosition(double position)
	{
		EController.setSetpoint(-position);
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
	
	private void elevatorPositionHold()
	{
		rightElevator.set(ESpeed);
		leftElevator.set(ESpeed);
		leftElevatorSlave.set(ESpeed);
	}

	//Driving forward using encoders
	private int driveForward(double ticks, boolean setFieldCentric)
	{
		//FRONT LEFT
		FLController.setSetpoint(-ticks);

		//FRONT RIGHT
		FRController.setSetpoint(ticks);

		//BACK LEFT
		BLController.setSetpoint(-ticks);

		//BACK RIGHT
		BRController.setSetpoint(ticks);
		
		avgSpeed = (FLSpeed + BLSpeed - FRSpeed - BRSpeed / 4);
		
		if(setFieldCentric)
		{
			angle = ahrs.getYaw();
		}else
		{
			//To drive straight without using field-centric
			angle = 0.0;
		}
		if(Math.abs(getTicks(backLeftDrive)) >= ticks - tolerance && Math.abs(getTicks(backLeftDrive)) <= ticks + tolerance)
		{
			resetWheelEncoders();
			Timer.delay(0.3);
			return 1;
		}else
		{
			drivetrain.driveCartesian(-0.1, avgSpeed, forwardPID(), -angle);
			return 0;
		}
	}

	//Driving left using encoders
	private int driveRight(double ticks, boolean setFieldCentric)
	{
		//FRONT LEFT
		FLController.setSetpoint(ticks);

		//FRONT RIGHT
		FRController.setSetpoint(ticks);

		//BACK LEFT
		BLController.setSetpoint(-ticks);

		//BACK RIGHT
		BRController.setSetpoint(-ticks);

		avgSpeed = (FLSpeed + BLSpeed - FRSpeed - BRSpeed / 4);
		
		if(setFieldCentric)
		{
			angle = ahrs.getYaw();
		}else
		{
			//To drive straight without using field-centric
			angle = 0.0;
		}
		
		if(Math.abs(getTicks(backLeftDrive)) >= ticks - tolerance && Math.abs(getTicks(backLeftDrive)) <= ticks + tolerance)
		{
			resetWheelEncoders();
			Timer.delay(0.3);
			return 1;
		}else
		{
			drivetrain.driveCartesian(avgSpeed, 0.0, forwardPID(), -angle);
			return 0;
		}
	}

	//Driving right using encoders
	private int driveLeft(double ticks, boolean setFieldCentric)
	{
		//FRONT LEFT
		FLController.setSetpoint(-ticks);

		//FRONT RIGHT
		FRController.setSetpoint(-ticks);

		//BACK LEFT
		BLController.setSetpoint(ticks);

		//BACK RIGHT
		BRController.setSetpoint(ticks);

		avgSpeed = (FLSpeed + BLSpeed - FRSpeed - BRSpeed / 4);
		
		if(setFieldCentric)
		{
			angle = ahrs.getYaw();
		}else
		{
			//To drive straight without using field-centric
			angle = 0.0;
		}
		
		if(Math.abs(getTicks(backRightDrive)) >= ticks - tolerance && Math.abs(getTicks(backRightDrive)) <= ticks + tolerance)
		{
			resetWheelEncoders();
			Timer.delay(0.3);
			return 1;
		}else
		{
			drivetrain.driveCartesian(avgSpeed, 0.0, forwardPID(), -angle);
			return 0;
		}
	}

	//Driving backward using encoders
	private int driveBackward(double ticks, boolean setFieldCentric)
	{
		//FRONT LEFT
		FLController.setSetpoint(ticks);

		//FRONT RIGHT
		FRController.setSetpoint(-ticks);

		//BACK LEFT
		BLController.setSetpoint(ticks);

		//BACK RIGHT
		BRController.setSetpoint(-ticks);

		avgSpeed = (FLSpeed + BLSpeed - FRSpeed - BRSpeed / 4);
		
		if(setFieldCentric)
		{
			angle = ahrs.getYaw();
		}else
		{
			//To drive straight without using field-centric
			angle = 0.0;
		}
		
		if(Math.abs(getTicks(backRightDrive)) >= ticks - tolerance && Math.abs(getTicks(backRightDrive)) <= ticks + tolerance)
		{
			resetWheelEncoders();
			Timer.delay(0.3);
			return 1;
		}else
		{
			drivetrain.driveCartesian(0.0, avgSpeed, forwardPID(), -angle);
			return 0;
		}
	}

	//Resets all encoders
	private void resetWheelEncoders()
	{
		frontLeftDrive.setSelectedSensorPosition(0, 0, 10);
		frontRightDrive.setSelectedSensorPosition(0, 0, 10);
		backLeftDrive.setSelectedSensorPosition(0, 0, 10);
		backRightDrive.setSelectedSensorPosition(0, 0, 10);
	}

	private void resetElevatorEncoder()
	{
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
		SmartDashboard.putNumber("Elevator Height", rightElevator.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("OPERATION", operation);
	}
}
