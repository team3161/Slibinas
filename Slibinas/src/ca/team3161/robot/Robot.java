package ca.team3161.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import ca.team3161.lib.utils.controls.LogitechDualAction;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.MecanumDrive;


public class Robot extends IterativeRobot implements PIDOutput 
{ 

	//This is declaring the Motors with the corresponding Controllers
	private WPI_TalonSRX backRightDrive = new WPI_TalonSRX(3);
	private WPI_TalonSRX frontRightDrive = new WPI_TalonSRX(1);
	private WPI_TalonSRX frontLeftDrive = new WPI_TalonSRX(0);
	private WPI_TalonSRX backLeftDrive = new WPI_TalonSRX(2);
	private VictorSP IntakeL = new VictorSP (0);
	private VictorSP IntakeR = new VictorSP (1);
	//Declaring the way the robot will drive - RoboDrive class
	private MecanumDrive drivetrain;
	//Declaring the AHRS class to get gyro headings
	private AHRS ahrs;
	double angle;

	//This is declaring both Controllers 
	private LogitechDualAction driverpad = new LogitechDualAction(0);
	private LogitechDualAction rotateStick = new LogitechDualAction(1);

	//For PID
	double rotateToAngleRate;
	double P = 0.02;
	double I = 0.00;
	double D = 0.06;
	double kToleranceDegrees = 2.0f;
	PIDController turnController;
	boolean rotateToAngle;
	double currentRotationRate;

	//Configures all the Joysticks and Buttons for both controllers
	double leftStickX;
	double leftStickY;	 
	double rightStickX;
	double rightStickY;
	boolean getButtonSTART;
	boolean getButtonY;
	boolean getButtonX;
	boolean getButtonA;
	boolean getButtonA_Operator;
	boolean getButtonB;
	boolean getButtonB_Operator;
	boolean getButtonLB_Operator;
	boolean getButtonLT_Operator;
	
	//need to set variables to use PCM
	private Compressor air = new Compressor(0);
	private boolean pressureSwitch;
	
	//Declaring Buttons for the pistons
	private DoubleSolenoid claw = new DoubleSolenoid (1,0);

	//Declaring a string to get the switch/scale positioning for each alliance
	String gameData;

	//Declaring a timer for autonomous timings
	Timer autoTimer = new Timer();;

	//Declaring positions for starting autonomous
	boolean MIDDLE, LEFT, RIGHT, DO_NOTHING;

	public void robotInit() 
	{
		//Reads in gyro readings for USB
		ahrs = new AHRS(SerialPort.Port.kUSB);
		//Initiate the RoboDrive class, so that drivetrain variable can be used with the talons - driving controller
		drivetrain = new MecanumDrive(frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive);
		//Resetting the gyro reading for the rest of run time
		ahrs.reset();
		//Executes PID calculations
		turnController = new PIDController(P, I, D, ahrs, this);
		turnController.setInputRange(-180.0f,  180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);
		//Initiate Encoders for all wheels
		//NEED TO CONFIGURE CHANNELS TO EACH ENCODER
		/*
		Encoder fLeft = new Encoder (0, 1, false, Encoder.EncodingType.k4X);
		Encoder fRight = new Encoder (0, 1, false, Encoder.EncodingType.k4X);
		Encoder bLeft = new Encoder (0, 1, false, Encoder.EncodingType.k4X);
		Encoder bRight = new Encoder (0, 1, false, Encoder.EncodingType.k4X);
		 */

		//The robot's claw is set to closed because it will be holding a cube when turned on
		ClawClose();
		
		//pneumatics variables
		pressureSwitch = air.getPressureSwitchValue();
	}

	public void autonomousInit() 
	{
		ahrs.reset();
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		MIDDLE = true;
	}

	public void autonomousPeriodic()
	{
		//get air for pneumatics
		if (pressureSwitch == false) {
			air.setClosedLoopControl(true);
		}
			
		
		frontLeftDrive.set(0.4 + forwardPID());
		backLeftDrive.set(0.55 - forwardPID());
		backRightDrive.set(-0.4 + forwardPID());
		frontRightDrive.set(-0.55 - forwardPID());
		
		//drivetrain.driveCartesian(0.0, 0.0, forwardPID(), 0.0);

		//attempt for the navx displacement methods FAIL INACCURATE

		/*if (ahrs.getDisplacementY() < 3.1){
			drivetrain.driveCartesian(-0.4, 0.0, forwardPID(), 0.0);
		} */


		//CASE 1
		//The robot drives 78" at half speed 4.02s to gain 1rp - 1.77 full speed

		/* if(MIDDLE && gameData.charAt(0) == 'L')
		{
			//DRIVE FORWARD 98"
			if(autoTimer.get() < 1.0 && autoTimer.get() > 0)
			{
				drivetrain.driveCartesian(-0.5, 0.0, forwardPID(), 0.0);
			}
		} */
		
		//stop taking air when pneumatics reaches 120 psi
				if (pressureSwitch == true) {
					air.setClosedLoopControl(false);
				}

	}

	public void teleopPeriodic() 
	{
		leftStickX = driverpad.getValue(LogitechDualAction.LogitechControl.LEFT_STICK, LogitechDualAction.LogitechAxis.X);
		leftStickY = driverpad.getValue(LogitechDualAction.LogitechControl.LEFT_STICK, LogitechDualAction.LogitechAxis.Y);
		rightStickX = driverpad.getValue(LogitechDualAction.LogitechControl.RIGHT_STICK, LogitechDualAction.LogitechAxis.X);
		rightStickY = rotateStick.getValue(LogitechDualAction.LogitechControl.RIGHT_STICK, LogitechDualAction.LogitechAxis.Y);
		getButtonSTART = driverpad.getButton(LogitechDualAction.LogitechButton.START);
		getButtonY = driverpad.getButton(LogitechDualAction.LogitechButton.Y);
		getButtonX = driverpad.getButton(LogitechDualAction.LogitechButton.X);
		getButtonA = driverpad.getButton(LogitechDualAction.LogitechButton.A);
		getButtonA_Operator = rotateStick.getButton(LogitechDualAction.LogitechButton.A);
		getButtonB_Operator = rotateStick.getButton(LogitechDualAction.LogitechButton.B);
		getButtonB = driverpad.getButton(LogitechDualAction.LogitechButton.B);
		getButtonLB_Operator = rotateStick.getButton(LogitechDualAction.LogitechButton.LEFT_BUMPER);
		getButtonLT_Operator = rotateStick.getButton(LogitechDualAction.LogitechButton.LEFT_TRIGGER);

		//get air for pneumatics
				if (pressureSwitch == false) {
					air.setClosedLoopControl(true);
				}
		
		//gets yaw from gyro continuously
		angle = ahrs.getYaw();

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
		if (getButtonSTART) {
			ahrs.reset();
		}
		if (getButtonY) {
			currentRotationRate = forwardPID();
			rotateToAngle = true;
		}
		else if (getButtonB) {
			currentRotationRate = rightPID();
			rotateToAngle = true;
		}
		else if (getButtonA) {
			currentRotationRate = backwardPID();
			rotateToAngle = true;
		}
		else if (getButtonX) {
			currentRotationRate = leftPID();
			rotateToAngle = true;
		}

		if(!rotateToAngle)
		{
			turnController.disable();
			currentRotationRate = rightStickX;
		}

		//Calls upon the mecanumDrive_Cartesian method that sends specific power to the talons
		//The rotation for the right stick is cut down by half
		drivetrain.driveCartesian (leftStickY,-leftStickX, currentRotationRate * 0.5, angle);

		//Setting speeds for the claw's motors to intake or shoot out the cube
		//Left Bumper intakes, Left Trigger spits out cube 
		if(getButtonLT_Operator)
		{
			ClawOutput();
		}
		else if (getButtonLB_Operator) 
		{
			ClawIntake();
		}else {		// The motors are always pulling in the cube so wont fly out when turning
			ClawStandby();
		}

		//Setting positions for the pistons
		//(A) closes the claw, (B) opens the claw
		if(getButtonA_Operator) 
		{
			ClawClose();
		}
		else if(getButtonB_Operator) 
		{
			ClawOpen();
		}
		
		//stop taking air when pneumatics reaches 120 psi
				if (pressureSwitch == true) {
					air.setClosedLoopControl(false);
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
		IntakeL.set(0.5);
		IntakeR.set(-0.5);
	}
	//Spits out the cube
	private void ClawOutput()
	{
		IntakeL.set(-0.5);
		IntakeR.set(0.5);
	}
	//Motors pulling cube in lightly
	private void ClawStandby()
	{
		IntakeL.set(0.15);
		IntakeR.set(-0.15);
	}

	private void ClawStopWheels()
	{
		IntakeL.set(0.0);
		IntakeR.set(0.0);
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

}
