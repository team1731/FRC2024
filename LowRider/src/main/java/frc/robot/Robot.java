// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.Scanner;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OpConstants;
import frc.robot.Constants.OpConstants.LedOption;
import frc.robot.util.log.LogWriter;
import frc.robot.util.log.MessageLog;
import frc.robot.subsystems.*;

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
  private boolean redAlliance;
  private int stationNumber = 0;
  public static long millis = System.currentTimeMillis();
  
  private CommandSwerveDrivetrain driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private WristSubsystem wristSubsystem;
  private IntakeShootStateMachine intakeShootStateMachine;
  private ClimbStateMachine climbStateMachine;
 

  public Robot() {
  }

  // SUBSYSTEM DECLARATION
  private LEDStringSubsystem ledSubsystem;

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
	DataLogManager.start();
	PortForwarder.add(5800, "photonvision.local", 5800);
	PortForwarder.add(1181, "photonvision.local", 1181);
	PortForwarder.add(1182, "photonvision.local", 1182);
	PortForwarder.add(1183, "photonvision.local", 1183);
	PortForwarder.add(1184, "photonvision.local", 1184);
	PortForwarder.add(1185, "photonvision.local", 1185);
	PortForwarder.add(1186, "photonvision.local", 1186);
	PortForwarder.add(1187, "photonvision.local", 1187);
//	LogWriter.setupLogging();
	MessageLog.start();
	System.out.println("\n\n\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  EVENT: " + DriverStation.getEventName() + " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n");

	LiveWindow.disableAllTelemetry();
	

    driveSubsystem = new CommandSwerveDrivetrain(true, TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);

	ledSubsystem = new LEDStringSubsystem(true);
	intakeSubsystem = new IntakeSubsystem(true);
	

	shooterSubsystem = new ShooterSubsystem(true);
	visionSubsystem = new VisionSubsystem(true, driveSubsystem);
    wristSubsystem = new WristSubsystem(true, visionSubsystem);
	intakeShootStateMachine = new IntakeShootStateMachine(intakeSubsystem, shooterSubsystem, ledSubsystem, visionSubsystem);
	elevatorSubsystem = new ElevatorSubsystem(true, wristSubsystem, intakeShootStateMachine);
	 climbStateMachine = new ClimbStateMachine(elevatorSubsystem, wristSubsystem, intakeShootStateMachine);
	// Instantiate our robot container. This will perform all of our button bindings,
	// and put our autonomous chooser on the dashboard
	m_robotContainer = new RobotContainer(driveSubsystem, shooterSubsystem, visionSubsystem, intakeSubsystem,  wristSubsystem, ledSubsystem, elevatorSubsystem,intakeShootStateMachine, climbStateMachine);

    wristSubsystem.retractTrapFlap();
	PathPlannerLogging.setLogActivePathCallback(null);
	Pose2d startingConfiguration = Robot.isRedAlliance()? new Pose2d(15.07,5.57, new Rotation2d(Math.toRadians(180))): new Pose2d(1.47,5.51, new Rotation2d (0));
	driveSubsystem.seedFieldRelative(startingConfiguration);
	Rotation2d operatorPerspective = Robot.isRedAlliance()? new Rotation2d(Math.toRadians(180)): new Rotation2d(Math.toRadians(0));
	driveSubsystem.setOperatorPerspectiveForward(operatorPerspective);
	
	
	initSubsystems();
    visionSubsystem.useVision(false);
	intakeShootStateMachine.turnOnLED();
	ledSubsystem.setColor(LedOption.INIT);
	//m_robotContainer.buildAuto10();
	String[] autoModes = RobotContainer.deriveAutoModes();
	for(String autoMode: autoModes){
	
		autoChooser.addOption(autoMode, autoMode);
		System.out.println("Added autoMode '" + autoMode + "' to autoChooser.");
	}
	autoChooser.setDefaultOption(Constants.AutoConstants.kAutoDefault, Constants.AutoConstants.kAutoDefault);
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
	SmartDashboard.updateValues();
	autoInitPreload();
  }
  

//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄ ▄██ ▄▄▄ ████ ▄▄▀██ ▄▄▄██ ▄▄▀███ ▄▄▀██ █████ ████▄ ▄█ ▄▄▀██ ▀██ ██ ▄▄▀██ ▄▄▄
//   ██ ███▄▄▄▀▀████ ▀▀▄██ ▄▄▄██ ██ ███ ▀▀ ██ █████ █████ ██ ▀▀ ██ █ █ ██ █████ ▄▄▄
//   █▀ ▀██ ▀▀▀ ████ ██ ██ ▀▀▀██ ▀▀ ███ ██ ██ ▀▀ ██ ▀▀ █▀ ▀█ ██ ██ ██▄ ██ ▀▀▄██ ▀▀▀
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  public static boolean isRedAlliance(){
	Optional<Alliance> alliance = DriverStation.getAlliance();
	if(alliance != null){
		return alliance.get() == DriverStation.Alliance.Red;
	}
	return false;
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

	System.out.println("\nPreloading AUTO CODE --> " + useCode);
	m_autonomousCommand = m_robotContainer.getNamedAutonomousCommand(useCode, redAlliance);
	if(m_autonomousCommand != null){
		autoCode = useCode;
		System.out.println("\n=====>>> PRELOADED AUTONOMOUS COMMAND: " + m_autonomousCommand);
	}
	else{
		System.out.println("\nAUTO CODE " + useCode + " IS NOT IMPLEMENTED -- STAYING WITH AUTO CODE " + autoCode);
	}
}


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄████ ▄▄▄ ██ ██ ██ ▄▄▀██ ▄▄▄ ██ ███ ██ ▄▄▄ █▄▄ ▄▄██ ▄▄▄██ ▄▀▄ ██ ▄▄▄ 
//   ██ ███ █ █ ██ ████ ██████▄▄▄▀▀██ ██ ██ ▄▄▀██▄▄▄▀▀██▄▀▀▀▄██▄▄▄▀▀███ ████ ▄▄▄██ █ █ ██▄▄▄▀▀
//   █▀ ▀██ ██▄ █▀ ▀███ ██████ ▀▀▀ ██▄▀▀▄██ ▀▀ ██ ▀▀▀ ████ ████ ▀▀▀ ███ ████ ▀▀▀██ ███ ██ ▀▀▀ 
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  private void initSubsystems() {
	ledSubsystem.init();
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
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    driveSubsystem.cleanUpOculusMessages();
	//m_robotContainer.displayEncoders();
  }


/** This function is called once each time the robot enters Disabled mode. */
//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   ██ ▄▄▀█▄ ▄██ ▄▄▄ █ ▄▄▀██ ▄▄▀██ █████ ▄▄▄██ ▄▄▀███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   ██ ██ ██ ███▄▄▄▀▀█ ▀▀ ██ ▄▄▀██ █████ ▄▄▄██ ██ ████ ███ █ █ ██ ████ ██
//   ██ ▀▀ █▀ ▀██ ▀▀▀ █ ██ ██ ▀▀ ██ ▀▀ ██ ▀▀▀██ ▀▀ ███▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void disabledInit() {
	ledSubsystem.setBlink(false);
	ledSubsystem.setColor(OpConstants.LedOption.INIT);
  }


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   ██ ▄▄▀█▄ ▄██ ▄▄▄ █ ▄▄▀██ ▄▄▀██ █████ ▄▄▄██ ▄▄▀████ ▄▄ ██ ▄▄▄██ ▄▄▀█▄ ▄██ ▄▄▄ ██ ▄▄▀█▄ ▄██ ▄▄▀
//   ██ ██ ██ ███▄▄▄▀▀█ ▀▀ ██ ▄▄▀██ █████ ▄▄▄██ ██ ████ ▀▀ ██ ▄▄▄██ ▀▀▄██ ███ ███ ██ ██ ██ ███ ███
//   ██ ▀▀ █▀ ▀██ ▀▀▀ █ ██ ██ ▀▀ ██ ▀▀ ██ ▀▀▀██ ▀▀ ████ █████ ▀▀▀██ ██ █▀ ▀██ ▀▀▀ ██ ▀▀ █▀ ▀██ ▀▀▄
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void disabledPeriodic() {
	String newCode = autoChooser.getSelected();
	if(newCode == null) newCode = Constants.AutoConstants.kAutoDefault;
	if(!newCode.equals(autoCode)) {
		System.out.println("New Auto Code read from dashboard. OLD: " + autoCode + ", NEW: " + newCode);
		autoInitPreload();
	}

	boolean redAlliance = Robot.isRedAlliance();
	if(this.redAlliance != redAlliance){
		this.redAlliance = redAlliance;
		System.out.println("\n\n===============>>>>>>>>>>>>>>  WE ARE " + (redAlliance?"RED":"BLUE") + " ALLIANCE  <<<<<<<<<<<<=========================");
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


/** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █ ▄▄▀██ ██ █▄▄ ▄▄██ ▄▄▄ ██ ▀██ ██ ▄▄▄ ██ ▄▀▄ ██ ▄▄▄ ██ ██ ██ ▄▄▄ ███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   █ ▀▀ ██ ██ ███ ████ ███ ██ █ █ ██ ███ ██ █ █ ██ ███ ██ ██ ██▄▄▄▀▀████ ███ █ █ ██ ████ ██
//   █ ██ ██▄▀▀▄███ ████ ▀▀▀ ██ ██▄ ██ ▀▀▀ ██ ███ ██ ▀▀▀ ██▄▀▀▄██ ▀▀▀ ███▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void autonomousInit() {
	    intakeShootStateMachine.startLEDs();
		visionSubsystem.useVision(false);
    	System.out.println("AUTO INIT");
		CommandScheduler.getInstance().cancelAll();

	if(m_autonomousCommand == null) {
		System.out.println("SOMETHING WENT WRONG - UNABLE TO RUN AUTONOMOUS! CHECK SOFTWARE!");
	}
	else {
        	System.out.println("------------> RUNNING AUTONOMOUS COMMAND: " + m_autonomousCommand + " <----------");
			m_autonomousCommand.schedule();
		}
    	System.out.println("autonomousInit: End");
  }


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █ ▄▄▀██ ██ █▄▄ ▄▄██ ▄▄▄ ██ ▀██ ██ ▄▄▄ ██ ▄▀▄ ██ ▄▄▄ ██ ██ ██ ▄▄▄ ████ ▄▄ ██ ▄▄▄██ ▄▄▀█▄ ▄██ ▄▄▄ ██ ▄▄▀█▄ ▄██ ▄▄▀
//   █ ▀▀ ██ ██ ███ ████ ███ ██ █ █ ██ ███ ██ █ █ ██ ███ ██ ██ ██▄▄▄▀▀████ ▀▀ ██ ▄▄▄██ ▀▀▄██ ███ ███ ██ ██ ██ ███ ███
//   █ ██ ██▄▀▀▄███ ████ ▀▀▀ ██ ██▄ ██ ▀▀▀ ██ ███ ██ ▀▀▀ ██▄▀▀▄██ ▀▀▀ ████ █████ ▀▀▀██ ██ █▀ ▀██ ▀▀▀ ██ ▀▀ █▀ ▀██ ▀▀▄
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void autonomousPeriodic() {
    if(doSD()){ System.out.println("AUTO PERIODIC");}
  }


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄▄ ▄▄██ ▄▄▄██ █████ ▄▄▄██ ▄▄▄ ██ ▄▄ ███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   ███ ████ ▄▄▄██ █████ ▄▄▄██ ███ ██ ▀▀ ████ ███ █ █ ██ ████ ██
//   ███ ████ ▀▀▀██ ▀▀ ██ ▀▀▀██ ▀▀▀ ██ ██████▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void teleopInit() {	

	intakeShootStateMachine.startLEDs();
	visionSubsystem.useVision(false);
	ledSubsystem.setColor(LedOption.GREEN);
	wristSubsystem.stopMoveWristToTarget();
    /* 
	intakeSubsystem.stopIntake();
	intakeSubsystem.stopJiggle();
	intakeSubsystem.stopFireNote();
	*/
	//driveSubsystem.seedFieldRelative(new Pose2d(new Translation2d(0,0), new Rotation2d(120)));
	
	// Record both DS control and joystick data in TELEOP
	MessageLog.getLogger();
	System.out.println("TELEOP INIT");
	CommandScheduler.getInstance().cancelAll();
	initSubsystems();
	intakeShootStateMachine.setCurrentInput(ISInput.STOP_INTAKE);
	intakeShootStateMachine.setCurrentInput(ISInput.STOP_SPEAKER);

	if (m_autonomousCommand != null) {
		m_autonomousCommand.cancel();
	}
	currentKeypadCommand = "";
	SmartDashboard.getString("keypadCommand", currentKeypadCommand);
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

	//if(doSD()){
	//	System.out.println("TELEOP PERIODIC");
	//}
		
  }


//   ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄
//   █▄▄ ▄▄██ ▄▄▄██ ▄▄▄ █▄▄ ▄▄███▄ ▄██ ▀██ █▄ ▄█▄▄ ▄▄
//   ███ ████ ▄▄▄██▄▄▄▀▀███ ██████ ███ █ █ ██ ████ ██
//   ███ ████ ▀▀▀██ ▀▀▀ ███ █████▀ ▀██ ██▄ █▀ ▀███ ██
//   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
	CommandScheduler.getInstance().cancelAll();
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
