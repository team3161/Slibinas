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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class Robot extends IterativeRobot implements PIDOutput 
{ 
	//This is declaring the Motors with the corresponding Controllers
	private WPI_TalonSRX frontLeftDrive = new WPI_TalonSRX(0);
	private WPI_TalonSRX frontRightDrive = new WPI_TalonSRX(1);
	private WPI_TalonSRX backLeftDrive = new WPI_TalonSRX(2);
	private WPI_TalonSRX backRightDrive = new WPI_TalonSRX(3);
	private VictorSP IntakeL = new VictorSP (3);
	private VictorSP IntakeR = new VictorSP (2);
	private VictorSP pivotL = new VictorSP (1);
	private VictorSP pivotR = new VictorSP (0);
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

	//For PID
	double rotateToAngleRate;
	double P = 0.02;
	double I = 0.00;
	double D = 0.06;
	float kToleranceDegrees = 2;
	PIDController turnController;
	boolean rotateToAngle;
	double rotateToHeight;
	double currentRotationRate;
	double height;

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
	boolean MIDDLE, LEFT, RIGHT, DO_NOTHING;

	//A counter for running the claw's intake after the claw closes
	private int c = 0;

	public void robotInit() 
	{
		frontLeftDrive.setInverted(false);
		frontRightDrive.setInverted(false);
		backRightDrive.setInverted(false);
		backLeftDrive.setInverted(false);

		//Reads in gyro readings from I2C connections
		ahrs = new AHRS(SPI.Port.kMXP);

		//Initiate the RoboDrive class, so that drive train variable can be used with the talons - driving controller
		drivetrain = new MecanumDrive(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive);

		//Resetting the gyro reading for the rest of run time
		ahrs.reset();

		//Executes PID calculations for drive train
		turnController = new PIDController(P, I, D, ahrs, this);
		turnController.setInputRange(-180.0f,  180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);

		//Initiate Encoders for all wheels
		//NEED TO CONFIGURE CHANNELS TO EACH ENCODER

		//Encoder Elevator_left = new Encoder (4, 1, false, Encoder.EncodingType.k4X);
		//Encoder Elevator_right = new Encoder (5,1, false, Encoder.EncodingType.k4X);

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
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		autoTimer.start();

		//for smartdashboard
		//autoCommand = (Command) autoChooser.getSelected();
		//autoCommand.start();
	}

	public void autonomousPeriodic()
	{

		//get air for pneumatics
		pressureSwitch = air.getPressureSwitchValue();
		if (!pressureSwitch) 
		{
			air.setClosedLoopControl(true);
		}

		if (autoTimer.get() < 4.0 && autoTimer.get() > 0.0){
			drivetrain.driveCartesian(0.0, 0.25, forwardPID(),0.0 );
		}	

		//CL test case
		if(autoTimer.get() < 4.67179 && autoTimer.get() > 2.25641)
		{
			drivetrain.driveCartesian(0.0, -0.3, forwardPID(), 0.0);
		}
		if(autoTimer.get() < 4.67179 && autoTimer.get() > 2.25641)
		{
			drivetrain.driveCartesian(0.3, 0.0, forwardPID(), 0.0);
		}
		if(autoTimer.get() < 7.9794 && autoTimer.get() > 5.2)
		{
			drivetrain.driveCartesian(0.0, -0.3, forwardPID(), 0.0);
		}
		if(autoTimer.get() < 9.0 && autoTimer.get() > 8.1)
		{
			ClawOutput();
		}if(autoTimer.get() < 12.4 && autoTimer.get() > 9.8)
		{
			drivetrain.driveCartesian(0.3, 0.0, forwardPID(), 0.0);
		}
		if(autoTimer.get() < 14.1 && autoTimer.get() > 12.7)
		{
			drivetrain.driveCartesian(0.0, -0.35, forwardPID(), 0.0);
		}

		//stop taking air when pneumatics reaches 120 psi
		if (pressureSwitch) 
		{
			air.setClosedLoopControl(false);
		}
	}

	public void teleopPeriodic() 
	{

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
			if(c <= 50)
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
			//height = leftElevator.get();
			ElevatorHold();
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
		rotateToAngleRate = output;
	}

	//Preset method that pushes out "right stick X rotation" with PID - backward
	private double forwardPID() 
	{
		turnController.setSetpoint(0.0f);
		turnController.enable();
		return rotateToAngleRate;
	}

	//Preset method that pushes out "right stick X rotation" with PID - forward
	private double backwardPID() 
	{
		turnController.setSetpoint(180.0f);
		turnController.enable();
		return rotateToAngleRate;
	}

	//Preset method that pushes out "right stick X rotation" with PID - right
	private double rightPID() 
	{
		turnController.setSetpoint(90.0f);
		turnController.enable();
		return rotateToAngleRate;
	}

	//Preset method that pushes out "right stick X rotation" with PID - left
	private double leftPID() 
	{
		turnController.setSetpoint(-90.0f);
		turnController.enable();
		return rotateToAngleRate;
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
		IntakeL.set(0.15);
		IntakeR.set(-0.15);
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
		pivotL.set(0.6);
		pivotR.set(-0.6);
	}

	//Claw Rotate Down
	private void ClawRotateDown()
	{
		pivotL.set(-0.6);
		pivotR.set(0.6);
	}

	//Claw motors are stopped
	private void ClawHold()
	{
		pivotL.set(0.0);
		pivotR.set(0.0);
	}
}
