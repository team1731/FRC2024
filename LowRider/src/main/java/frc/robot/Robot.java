// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.OptionalInt;
import java.util.Scanner;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.LogConstants;
import frc.robot.Constants.OpConstants;
import frc.robot.Constants.OpConstants.LedOption;
import frc.robot.util.log.LogWriter;
import frc.robot.util.log.MessageLog;
import frc.robot.subsystems.*;
import frc.robot.CommandSwerveDrivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  private Command m_autonomousCommand;
  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private String autoCode;
  private String currentKeypadCommand = "";
  private boolean isRedAlliance;
  private int stationNumber = 0;
  public static long millis = System.currentTimeMillis();
  private CommandSwerveDrivetrain driveSubsystem;
  private PoseEstimatorSubsystem s_poseEstimatorSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intake_subsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private WristSubsystem wristSubsystem;

  private boolean enabled = true;

  public Robot() {
  }

  // SUBSYSTEM DECLARATION
  private LEDStringSubsystem m_ledstring;
  private boolean ledBlinking;
  //private boolean armEmergencyStatus = false;

  // NOTE: FOR TESTING PURPOSES ONLY!
  //private final Joystick driver = new Joystick(0);
  //private final JoystickButton blinker = null; //new JoystickButton(driver, XboxController.Button.kX.value);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   * 
   * NOTE: ASCII ART from https://textfancy.com/text-art/  ("small negative")
   */
//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   ██ ▄▄▀██ ▄▄▄ ██ ▄▄▀██ ▄▄▄ █▄▄ ▄▄███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   ██ ▀▀▄██ ███ ██ ▄▄▀██ ███ ███ ██████ ███ █ █ ██ ████ ██
//   ██ ██ ██ ▀▀▀ ██ ▀▀ ██ ▀▀▀ ███ █████▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void robotInit() {
	
	LogWriter.setupLogging();
	MessageLog.start();
	System.out.println("\n\n\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  EVENT: " + DriverStation.getEventName() + " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n");

	LiveWindow.disableAllTelemetry();
	// PortForwarder.add(5800, "10.17.31.11", 5800);
	// PortForwarder.add(5801, "10.17.31.11", 5801);
	// PortForwarder.add(5802, "10.17.31.11", 5802);
	// PortForwarder.add(5803, "10.17.31.11", 5803);
	// PortForwarder.add(5804, "10.17.31.11", 5804);

	driveSubsystem = TunerConstants.DriveTrain; // My drivetrain
	driveSubsystem.setEnabled(true);
	//s_Swerve = new Swerve(false);
	s_poseEstimatorSubsystem = new PoseEstimatorSubsystem(false);
	m_ledstring = new LEDStringSubsystem(false);
	intake_subsystem = new IntakeSubsystem(false);
	wristSubsystem = new WristSubsystem(false);
	elevatorSubsystem = new ElevatorSubsystem(false, wristSubsystem, intake_subsystem);
	shooterSubsystem = new ShooterSubsystem(false);

	// Instantiate our robot container. This will perform all of our button bindings,
	// and put our autonomous chooser on the dashboard
	m_robotContainer = new RobotContainer(driveSubsystem, shooterSubsystem, s_poseEstimatorSubsystem, intake_subsystem,  wristSubsystem, m_ledstring, elevatorSubsystem); //, s_poseEstimatorSubsystem), s_armSubSystem, m_ledstring);


	PathPlannerLogging.setLogActivePathCallback(null); //.setLoggingCallbacks(null, s_Swerve::logPose, null, s_Swerve::defaultLogError);

	initSubsystems();
	//s_armSubSystem.resetArmEncoders();

	String[] autoModes = RobotContainer.deriveAutoModes();
	for(String autoMode: autoModes){
		autoChooser.addOption(autoMode, autoMode);
		System.out.println("Added autoMode '" + autoMode + "' to autoChooser.");
	}
    SmartDashboard.putData(AutoConstants.kAutoCodeKey, autoChooser);
	SmartDashboard.putString("Build Info - Branch", "N/A");
	SmartDashboard.putString("Build Info - Commit Hash", "N/A");
	SmartDashboard.putString("Build Info - Date", "N/A");

	/*
	 * Note: do not think this is implemented in the gradle build, if we want to print this we will need to carry that over
	 */
	try {
		File buildInfoFile = new File(Filesystem.getDeployDirectory(), "DeployedBranchInfo.txt");
		if(buildInfoFile.exists() && buildInfoFile.canRead()){
			Scanner reader = new Scanner(buildInfoFile);
			int i = 0;
			while(reader.hasNext()){
				if(i == 0){
					SmartDashboard.putString("Build Info - Branch", reader.nextLine());
				} else if(i == 1){
					SmartDashboard.putString("Build Info - Commit Hash", reader.nextLine());
				} else {
					SmartDashboard.putString("Build Info - Date", reader.nextLine());
				}
				i++;
			}			
			reader.close();
		}
	} catch (FileNotFoundException fnf) {
		System.err.println("DeployedBranchInfo.txt not found");
		fnf.printStackTrace();
	}

	autoInitPreload();

	//For testing LED Blinking only. The arm will set blink true after a piece has been secured.
	//ledBlinking = true;
	//blinker.onTrue(new InstantCommand(() -> {m_ledstring.setBlink(ledBlinking); ledBlinking = !ledBlinking;}));
  }
  

//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄ ▄██ ▄▄▄ ████ ▄▄▀██ ▄▄▄██ ▄▄▀███ ▄▄▀██ █████ ████▄ ▄█ ▄▄▀██ ▀██ ██ ▄▄▀██ ▄▄▄
//   ██ ███▄▄▄▀▀████ ▀▀▄██ ▄▄▄██ ██ ███ ▀▀ ██ █████ █████ ██ ▀▀ ██ █ █ ██ █████ ▄▄▄
//   █▀ ▀██ ▀▀▀ ████ ██ ██ ▀▀▀██ ▀▀ ███ ██ ██ ▀▀ ██ ▀▀ █▀ ▀█ ██ ██ ██▄ ██ ▀▀▄██ ▀▀▀
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  private boolean isRedAlliance(){
	return false; //DriverStation.getAlliance().equals(DriverStation.Alliance.Red);
  }


// ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// ██ ▄▄ ██ ▄▄▄█▄▄ ▄▄████ ▄▄▄ █▄▄ ▄▄█ ▄▄▀█▄▄ ▄▄█▄ ▄██ ▄▄▄ ██ ▀██ ████ ▀██ ██ ██ ██ ▄▀▄ ██ ▄▄▀██ ▄▄▄██ ▄▄▀
// ██ █▀▀██ ▄▄▄███ ██████▄▄▄▀▀███ ███ ▀▀ ███ ████ ███ ███ ██ █ █ ████ █ █ ██ ██ ██ █ █ ██ ▄▄▀██ ▄▄▄██ ▀▀▄
// ██ ▀▀▄██ ▀▀▀███ ██████ ▀▀▀ ███ ███ ██ ███ ███▀ ▀██ ▀▀▀ ██ ██▄ ████ ██▄ ██▄▀▀▄██ ███ ██ ▀▀ ██ ▀▀▀██ ██ 
// ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  private OptionalInt getStationNumber(){
	return DriverStation.getLocation();
  }


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █ ▄▄▀██ ██ █▄▄ ▄▄██ ▄▄▄ ███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄████ ▄▄ ██ ▄▄▀██ ▄▄▄██ █████ ▄▄▄ █ ▄▄▀██ ▄▄▀
//   █ ▀▀ ██ ██ ███ ████ ███ ████ ███ █ █ ██ ████ ██████ ▀▀ ██ ▀▀▄██ ▄▄▄██ █████ ███ █ ▀▀ ██ ██ 
//   █ ██ ██▄▀▀▄███ ████ ▀▀▀ ███▀ ▀██ ██▄ █▀ ▀███ ██████ █████ ██ ██ ▀▀▀██ ▀▀ ██ ▀▀▀ █ ██ ██ ▀▀ 
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  private void autoInitPreload() {
	m_autonomousCommand = null;

	String useCode = autoChooser.getSelected();
	if(useCode == null){
		useCode = (autoCode == null ? Constants.AutoConstants.kAutoDefault : autoCode);
	}
	boolean isRedAlliance = isRedAlliance();
	if( !useCode.startsWith("Red_") && !useCode.startsWith("Blu_")){
		useCode = (isRedAlliance ? "Red_" : "Blu_") + useCode;
	}
	System.out.println("\nPreloading AUTO CODE --> " + useCode);
	m_autonomousCommand = m_robotContainer.getNamedAutonomousCommand(useCode, isRedAlliance);
	if(m_autonomousCommand != null){
		autoCode = useCode;
		System.out.println("\n=====>>> PRELOADED AUTONOMOUS COMMAND: " + m_autonomousCommand);
	}
	else{
		System.out.println("\nAUTO CODE " + useCode + " IS NOT IMPLEMENTED -- STAYING WITH AUTO CODE " + autoCode);
	}

	System.out.println("\nAUTO CODE being used by the software --> " + autoCode + "\n");
	}


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄████ ▄▄▄ ██ ██ ██ ▄▄▀██ ▄▄▄ ██ ███ ██ ▄▄▄ █▄▄ ▄▄██ ▄▄▄██ ▄▀▄ ██ ▄▄▄ 
//   ██ ███ █ █ ██ ████ ██████▄▄▄▀▀██ ██ ██ ▄▄▀██▄▄▄▀▀██▄▀▀▀▄██▄▄▄▀▀███ ████ ▄▄▄██ █ █ ██▄▄▄▀▀
//   █▀ ▀██ ██▄ █▀ ▀███ ██████ ▀▀▀ ██▄▀▀▄██ ▀▀ ██ ▀▀▀ ████ ████ ▀▀▀ ███ ████ ▀▀▀██ ███ ██ ▀▀▀ 
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  private void initSubsystems() {
	if (enabled) {
		m_ledstring.init();
  	}
  }
  

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
// ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
// ██ ▄▄▀██ ▄▄▄ ██ ▄▄▀██ ▄▄▄ █▄▄ ▄▄████ ▄▄ ██ ▄▄▄██ ▄▄▀█▄ ▄██ ▄▄▄ ██ ▄▄▀█▄ ▄██ ▄▄▀
// ██ ▀▀▄██ ███ ██ ▄▄▀██ ███ ███ ██████ ▀▀ ██ ▄▄▄██ ▀▀▄██ ███ ███ ██ ██ ██ ███ ███
// ██ ██ ██ ▀▀▀ ██ ▀▀ ██ ▀▀▀ ███ ██████ █████ ▀▀▀██ ██ █▀ ▀██ ▀▀▀ ██ ▀▀ █▀ ▀██ ▀▀▄
// ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void robotPeriodic() {
	if (enabled){
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    	CommandScheduler.getInstance().run();

	//
	// read the KEYPAD, write to NETWORK TABLES
	//
		// String newKeypadEntry = keypad.getEntry("driver entry").getString(oldKeypadEntry);
		// if (!newKeypadEntry.equals(oldKeypadEntry)){
        // 	System.out.println(".\n.\n.\nDRIVER ENTRY ==========================>>>>>>>> " + newKeypadEntry + "\n.\n.\n.");
		// 	oldKeypadEntry = newKeypadEntry;
		// 	SmartDashboard.putString("keypadCommand", newKeypadEntry);
		// 	m_robotContainer.processKeypadCommand(newKeypadEntry);
		// // sm_armStateMachine.setOperatorSequence(newKeypadEntry);
		// }

		m_robotContainer.displayEncoders();
	}
  }


/** This function is called once each time the robot enters Disabled mode. */
//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   ██ ▄▄▀█▄ ▄██ ▄▄▄ █ ▄▄▀██ ▄▄▀██ █████ ▄▄▄██ ▄▄▀███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   ██ ██ ██ ███▄▄▄▀▀█ ▀▀ ██ ▄▄▀██ █████ ▄▄▄██ ██ ████ ███ █ █ ██ ████ ██
//   ██ ▀▀ █▀ ▀██ ▀▀▀ █ ██ ██ ▀▀ ██ ▀▀ ██ ▀▀▀██ ▀▀ ███▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void disabledInit() {
  }


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   ██ ▄▄▀█▄ ▄██ ▄▄▄ █ ▄▄▀██ ▄▄▀██ █████ ▄▄▄██ ▄▄▀████ ▄▄ ██ ▄▄▄██ ▄▄▀█▄ ▄██ ▄▄▄ ██ ▄▄▀█▄ ▄██ ▄▄▀
//   ██ ██ ██ ███▄▄▄▀▀█ ▀▀ ██ ▄▄▀██ █████ ▄▄▄██ ██ ████ ▀▀ ██ ▄▄▄██ ▀▀▄██ ███ ███ ██ ██ ██ ███ ███
//   ██ ▀▀ █▀ ▀██ ▀▀▀ █ ██ ██ ▀▀ ██ ▀▀ ██ ▀▀▀██ ▀▀ ████ █████ ▀▀▀██ ██ █▀ ▀██ ▀▀▀ ██ ▀▀ █▀ ▀██ ▀▀▄
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void disabledPeriodic() {
	if (enabled){
		//s_armSubSystem.resetArmEncoders();
    	if (System.currentTimeMillis() % 5000 == 0) {
			// SmartDashboard.putBoolean("LowSensor", m_sequencer.lowSensorHasBall());
			// SmartDashboard.putBoolean("MidSensor", m_sequencer.midSensorHasBall());
			// SmartDashboard.putBoolean("HighSensor", m_sequencer.highSensorHasBall());
		}

		// if(s_armSubSystem.isInEncodersOutOfBoundsCondition()) {
		// 	m_ledstring.setColor(OpConstants.LedOption.RED);
		// } else {
		// 	m_ledstring.setColor(OpConstants.LedOption.GREEN);
		// }

		String newCode = autoChooser.getSelected();
		if(newCode == null) newCode = Constants.AutoConstants.kAutoDefault;
		if(!newCode.equals(autoCode)) {
    	    System.out.println("New Auto Code read from dashboard. OLD: " + autoCode + ", NEW: " + newCode);
			autoInitPreload();
		}

		boolean isRedAlliance = isRedAlliance();
		if(this.isRedAlliance != isRedAlliance){
			this.isRedAlliance = isRedAlliance;
    	    System.out.println("\n\n===============>>>>>>>>>>>>>>  WE ARE " + (isRedAlliance?"RED":"BLUE") + " ALLIANCE  <<<<<<<<<<<<=========================");
			this.autoInitPreload();
		}

        if(Robot.isReal()){
			try{
				OptionalInt stationNumberInt = getStationNumber();
				if(stationNumberInt.isPresent()) {
					int stationNumber = stationNumberInt.getAsInt();
					if(this.stationNumber != stationNumber){
						this.stationNumber = stationNumber;
						System.out.println("===============>>>>>>>>>>>>>>  WE ARE STATION NUMBER " + stationNumber + "  <<<<<<<<<<<<=========================\n");
					}
				}
			} catch (Exception e){
				System.out.println("Exception caught while looking for station number! == " + e);
			}
	    }
	}
  }


/** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █ ▄▄▀██ ██ █▄▄ ▄▄██ ▄▄▄ ██ ▀██ ██ ▄▄▄ ██ ▄▀▄ ██ ▄▄▄ ██ ██ ██ ▄▄▄ ███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   █ ▀▀ ██ ██ ███ ████ ███ ██ █ █ ██ ███ ██ █ █ ██ ███ ██ ██ ██▄▄▄▀▀████ ███ █ █ ██ ████ ██
//   █ ██ ██▄▀▀▄███ ████ ▀▀▀ ██ ██▄ ██ ▀▀▀ ██ ███ ██ ▀▀▀ ██▄▀▀▄██ ▀▀▀ ███▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void autonomousInit() {
	if (enabled){
    	System.out.println("AUTO INIT");
		CommandScheduler.getInstance().cancelAll();
		// s_armSubSystem.resetArmEncodersForAuto();

	if(m_autonomousCommand == null) {
		System.out.println("SOMETHING WENT WRONG - UNABLE TO RUN AUTONOMOUS! CHECK SOFTWARE!");
	}
	else {
        	System.out.println("------------> RUNNING AUTONOMOUS COMMAND: " + m_autonomousCommand + " <----------");
			m_robotContainer.zeroHeading();
			m_ledstring.setColor(OpConstants.LedOption.WHITE); // reset color to default from red/green set during disabled
			m_autonomousCommand.schedule();
		}
    	System.out.println("autonomousInit: End");
  	}
}


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █ ▄▄▀██ ██ █▄▄ ▄▄██ ▄▄▄ ██ ▀██ ██ ▄▄▄ ██ ▄▀▄ ██ ▄▄▄ ██ ██ ██ ▄▄▄ ████ ▄▄ ██ ▄▄▄██ ▄▄▀█▄ ▄██ ▄▄▄ ██ ▄▄▀█▄ ▄██ ▄▄▀
//   █ ▀▀ ██ ██ ███ ████ ███ ██ █ █ ██ ███ ██ █ █ ██ ███ ██ ██ ██▄▄▄▀▀████ ▀▀ ██ ▄▄▄██ ▀▀▄██ ███ ███ ██ ██ ██ ███ ███
//   █ ██ ██▄▀▀▄███ ████ ▀▀▀ ██ ██▄ ██ ▀▀▀ ██ ███ ██ ▀▀▀ ██▄▀▀▄██ ▀▀▀ ████ █████ ▀▀▀██ ██ █▀ ▀██ ▀▀▀ ██ ▀▀ █▀ ▀██ ▀▀▄
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void autonomousPeriodic() {
	if (enabled){
    	if(doSD()){ System.out.println("AUTO PERIODIC");}
	}
  }


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄▄ ▄▄██ ▄▄▄██ █████ ▄▄▄██ ▄▄▄ ██ ▄▄ ███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   ███ ████ ▄▄▄██ █████ ▄▄▄██ ███ ██ ▀▀ ████ ███ █ █ ██ ████ ██
//   ███ ████ ▀▀▀██ ▀▀ ██ ▀▀▀██ ▀▀▀ ██ ██████▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void teleopInit() {	
	if (enabled){
		// Record both DS control and joystick data in TELEOP
		MessageLog.getLogger();
    	System.out.println("TELEOP INIT");
		CommandScheduler.getInstance().cancelAll();
		initSubsystems();
		//for testing only
		m_ledstring.setColor(LedOption.INIT);
		// sm_armStateMachine.setIsInAuto(false);
		// sm_armStateMachine.initializeArm();
    	// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		currentKeypadCommand = "";
		SmartDashboard.getString("keypadCommand", currentKeypadCommand);
		// s_poseEstimatorSubsystem.disableVisionCorrection();   // if for some reason auto does not put it back in driver mode
  	}
  }

//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   ██ ▄▄▀██ ▄▄▄ ████ ▄▄▄ ██ ▄▄▀
//   ██ ██ ██ ███ ████▄▄▄▀▀██ ██ 
//   ██ ▀▀ ██ ▀▀▀ ████ ▀▀▀ ██ ▀▀ 
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  public static boolean doSD() {
	long now = System.currentTimeMillis();
	if (now - millis > 1000) {
		MessageLog.getLogger().flush();
		millis = now;
		return true;
	}
	return false;
  }

/** This function is called periodically during operator control. */
//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄▄ ▄▄██ ▄▄▄██ █████ ▄▄▄██ ▄▄▄ ██ ▄▄ ████ ▄▄ ██ ▄▄▄██ ▄▄▀█▄ ▄██ ▄▄▄ ██ ▄▄▀█▄ ▄██ ▄▄▀
//   ███ ████ ▄▄▄██ █████ ▄▄▄██ ███ ██ ▀▀ ████ ▀▀ ██ ▄▄▄██ ▀▀▄██ ███ ███ ██ ██ ██ ███ ███
//   ███ ████ ▀▀▀██ ▀▀ ██ ▀▀▀██ ▀▀▀ ██ ███████ █████ ▀▀▀██ ██ █▀ ▀██ ▀▀▀ ██ ▀▀ █▀ ▀██ ▀▀▄
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void teleopPeriodic() {
	if (enabled){
		//System.out.println("Setting the color");
		//m_ledstring.setColor(LedOption.INIT);
    	if(doSD()){
		//	System.out.println("TELEOP PERIODIC");

		}
		
    	String newKeypadCommand = SmartDashboard.getString("keypadCommand", currentKeypadCommand);
		if(!newKeypadCommand.equals(currentKeypadCommand)){
			// FEED FSM
			m_robotContainer.processKeypadCommand(newKeypadCommand);
			// sm_armStateMachine.setOperatorSequence(newKeypadCommand);
			currentKeypadCommand = newKeypadCommand;
		}
	}

	/*
	 * Change LED blinking status depending on whether holding a game piece or not
	 */
	if(!ledBlinking /* && sm_armStateMachine.isHoldingGamePiece() */) {
		m_ledstring.setBlink(true);
		ledBlinking = true;
	} else if(ledBlinking /* && !sm_armStateMachine.isHoldingGamePiece() */) {
		m_ledstring.setBlink(false);
		ledBlinking = false;
	}

	/*
	 * Change LED to indicate emergency status entry or exit
	 */
	// if(!armEmergencyStatus && sm_armStateMachine.isInEmergencyRecovery()) {
	// 	armEmergencyStatus = true;
	// 	m_ledstring.setBlink(false);
	// 	m_ledstring.setColor(OpConstants.LedOption.RED);
	// } else if(armEmergencyStatus && !sm_armStateMachine.isInEmergencyRecovery()) {
	// 	armEmergencyStatus = false;
	// 	// if the SM has a record of a game piece setting revert to that color, otherwise just go to default color
	// 	if(sm_armStateMachine.getGamePiece() == GamePiece.CONE) {
	// 		m_ledstring.setColor(OpConstants.LedOption.YELLOW);
	// 	} else if(sm_armStateMachine.getGamePiece() == GamePiece.CUBE) {
	// 		m_ledstring.setColor(OpConstants.LedOption.PURPLE);
	// 	} else {
	// 		m_ledstring.setColor(OpConstants.LedOption.WHITE);
	// 	}
	// }
  }


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄▄ ▄▄██ ▄▄▄██ ▄▄▄ █▄▄ ▄▄███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   ███ ████ ▄▄▄██▄▄▄▀▀███ ██████ ███ █ █ ██ ████ ██
//   ███ ████ ▀▀▀██ ▀▀▀ ███ █████▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    if (enabled){
		CommandScheduler.getInstance().cancelAll();
  	}
  }


  /** This function is called periodically during test mode. */
//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄▄ ▄▄██ ▄▄▄██ ▄▄▄ █▄▄ ▄▄████ ▄▄ ██ ▄▄▄██ ▄▄▀█▄ ▄██ ▄▄▄ ██ ▄▄▀█▄ ▄██ ▄▄▀
//   ███ ████ ▄▄▄██▄▄▄▀▀███ ██████ ▀▀ ██ ▄▄▄██ ▀▀▄██ ███ ███ ██ ██ ██ ███ ███
//   ███ ████ ▀▀▀██ ▀▀▀ ███ ██████ █████ ▀▀▀██ ██ █▀ ▀██ ▀▀▀ ██ ▀▀ █▀ ▀██ ▀▀▄
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void testPeriodic() {

  }

}
