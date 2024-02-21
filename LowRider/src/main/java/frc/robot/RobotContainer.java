// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  /* Controllers */
  private final CommandXboxController xboxController = new CommandXboxController(0);
  private final CommandXboxController xboxOperatorController = new CommandXboxController(1);


  /* Drive Controls */
  //private final int translationAxis = XboxController.Axis.kLeftY.value;
  //private final int strafeAxis = XboxController.Axis.kLeftX.value;
  //private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final Trigger kStart = xboxController.start();
  private final Trigger kBack = xboxController.back();
  private final Trigger ky = xboxController.y();
  private final Trigger kb = xboxController.b();
  private final Trigger ka = xboxController.a();
  private final Trigger kx = xboxController.x();
  private final Trigger kLeftBumper = xboxController.leftBumper();
  private final Trigger kRightBumper = xboxController.rightBumper();
  private final Trigger kLeftTrigger = xboxController.leftTrigger();
  private final Trigger kRightTrigger = xboxController.rightTrigger();

  /* Operator Buttons */

  private final Trigger operatorkStart = xboxOperatorController.start();
  private final Trigger operatorBack = xboxOperatorController.back();
  private final Trigger operatorky = xboxOperatorController.y();
  private final Trigger operatorkb = xboxOperatorController.b();
  private final Trigger operatorka = xboxOperatorController.a();
  private final Trigger operatorkx = xboxOperatorController.x();
  private final Trigger operatorkLeftBumper = xboxOperatorController.leftBumper();
  private final Trigger operatorkRightBumper = xboxOperatorController.rightBumper();
  private final Trigger operatorkLeftTrigger = xboxOperatorController.leftTrigger();
  private final Trigger operatorkRightTrigger = xboxOperatorController.rightTrigger();



  /* Subsystems */
  private CommandSwerveDrivetrain driveSubsystem;
  private PoseEstimatorSubsystem s_poseEstimatorSubsystem;
  private IntakeSubsystem s_intakeSubsystem;
  private WristSubsystem s_wristSubsystem;
  private final LEDStringSubsystem m_ledstring;
  private ShooterSubsystem s_ShooterSubsystem;
  private ElevatorSubsystem elevatorSubsystem;

  /* Auto Paths */
  private static HashMap<String, String> autoPaths;


  // The container for the robot. Contains subsystems, OI devices, and commands. 
  public RobotContainer(
          CommandSwerveDrivetrain driveSubsystem,
          ShooterSubsystem shooterSubsystem,
          PoseEstimatorSubsystem poseEstimatorSubsystem,
          IntakeSubsystem intakeSubsystem,
          WristSubsystem wristSubsystem,
          LEDStringSubsystem m_ledstring,
          ElevatorSubsystem elevatorSubsystem
          ) {
    

    this.driveSubsystem = driveSubsystem;
    s_ShooterSubsystem = shooterSubsystem;
    s_intakeSubsystem = intakeSubsystem;
    s_wristSubsystem = wristSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    s_poseEstimatorSubsystem = poseEstimatorSubsystem;


    this.m_ledstring = m_ledstring;

    if(driveSubsystem.isEnabled()){
      //NamedCommands.registerCommand("RotateLeft", new SequentialCommandGroup(s_Swerve.rotateRelative(-45.0) ));
      //NamedCommands.registerCommand("RotateRight", new SequentialCommandGroup(s_Swerve.rotateRelative(-45.0) ));
      NamedCommands.registerCommand("Intake", new SequentialCommandGroup(new AutoIntake(intakeSubsystem, wristSubsystem) ));
      NamedCommands.registerCommand("StartShooter", new SequentialCommandGroup(new AutoStartShooter(s_ShooterSubsystem) ));
      NamedCommands.registerCommand("StopShooter", new SequentialCommandGroup(new AutoStopShooter(s_ShooterSubsystem) ));
      NamedCommands.registerCommand("SetWristNote1", new SequentialCommandGroup(new InstantCommand(() ->  s_wristSubsystem.moveWrist(20)) ));
      NamedCommands.registerCommand("SetWristNote2", new SequentialCommandGroup(new InstantCommand(() ->  s_wristSubsystem.moveWrist(0)) ));
      NamedCommands.registerCommand("SetWristNote3", new SequentialCommandGroup(new InstantCommand(() ->  s_wristSubsystem.moveWrist(10)) ));
      NamedCommands.registerCommand("SetWristLongShot", new SequentialCommandGroup(new InstantCommand(() ->  s_wristSubsystem.moveWrist(19)) ));
      NamedCommands.registerCommand("SetWristLineShot", new SequentialCommandGroup(new InstantCommand(() ->  s_wristSubsystem.moveWrist(10)) ));
      NamedCommands.registerCommand("FireNote", new SequentialCommandGroup(new AutoFireNote( s_intakeSubsystem, s_ShooterSubsystem) ));
    }

    // Configure the button bindings
    configureButtonBindings();

  }
   

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /*
     * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     * DRIVER BUTTONS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     */
    if (driveSubsystem.isEnabled()) {
      driveSubsystem.setDefaultCommand( // Drivetrain will execute this command periodically
          driveSubsystem.applyRequest(
              () -> drive.withVelocityX(-(Math.abs(xboxController.getLeftY()) * xboxController.getLeftY()) * MaxSpeed)                                                                                                                     
                  .withVelocityY(-(Math.abs(xboxController.getLeftX()) * xboxController.getLeftX()) * MaxSpeed) 
                  .withRotationalRate(-xboxController.getRightX() * MaxAngularRate) 
                                                                                    
          ));
    }

    kLeftTrigger.whileTrue(new IntakeCommand(s_intakeSubsystem, s_wristSubsystem));
    kRightTrigger.whileTrue(new FireNoteSpeakerCommand(s_intakeSubsystem, s_ShooterSubsystem));

    kRightBumper.whileTrue(new AmpScoringCommand(s_intakeSubsystem, elevatorSubsystem, s_wristSubsystem));
    kLeftBumper.whileTrue(new ClimbCommand(s_intakeSubsystem, s_ShooterSubsystem, elevatorSubsystem, s_wristSubsystem));
    kx.whileTrue(new TrapScoringCommand(s_intakeSubsystem, elevatorSubsystem, s_wristSubsystem));

    ka.onTrue(new InstantCommand(() -> s_wristSubsystem.retractTrapFlap()));
    kb.onTrue(new InstantCommand(() -> s_wristSubsystem.extendTrapFlap()));

    kStart.onTrue(driveSubsystem.runOnce(() -> driveSubsystem.seedFieldRelative()));

    operatorkLeftBumper.onTrue(new InstantCommand(() -> {
      s_ShooterSubsystem.shoot();
    }));
    operatorkRightBumper.onTrue(new InstantCommand(() -> {
      s_ShooterSubsystem.stopShooting();
    }));
    operatorkStart
        .onTrue(new InstantCommand(() -> s_intakeSubsystem.reverseFeed()))
        .onFalse(new InstantCommand(() -> s_intakeSubsystem.stopFeed()));

    // Far Shot
    operatorky.onTrue(new InstantCommand(() -> s_wristSubsystem.moveWrist(15)))
        .onFalse(new InstantCommand(() -> s_wristSubsystem.moveWrist(0)));
    // Close Shot
    operatorkb.onTrue(new InstantCommand(() -> s_wristSubsystem.moveWrist(25)))
        .onFalse(new InstantCommand(() -> s_wristSubsystem.moveWrist(0)));

    driveSubsystem.registerTelemetry(logger::telemeterize);
  }

  public static String[] deriveAutoModes() {
    autoPaths = findPaths(new File(Filesystem.getLaunchDirectory(), (Robot.isReal() ? "home/lvuser" : "src/main") + "/deploy/pathplanner/autos"));
    List<String> autoModes = new ArrayList<String>();
    for(String key : autoPaths.keySet()){
      if(!autoModes.contains(key)){
        autoModes.add(key);
      }
    }
    autoModes.sort((p1, p2) -> p1.compareTo(p2));
    return autoModes.toArray(String[]::new);
  }

  private static HashMap<String, String> findPaths(File directory){
    HashMap<String, String> autoPaths = new HashMap<String, String>();
    if(!directory.exists()){
      System.out.println("FATAL: path directory not found! " + directory.getAbsolutePath());
    }
    else {
      File[] files = directory.listFiles();
      if(files == null){
        System.out.println("FATAL: I/O error or NOT a directory: " + directory);
      }
      else
      {
        for (File file : files) {
            String fileName = file.getName();
            if ((fileName.startsWith("Blu") || fileName.startsWith("Red")) && fileName.endsWith(".auto")) {
              String key = fileName.replace(".auto", "");
              String path = file.getAbsolutePath();
              System.out.println(path);
                autoPaths.put(key, path);
            }
        }
      }
    }
    return autoPaths;
  }

  public Command getNamedAutonomousCommand(String autoName, boolean isRedAlliance) {
    String alliancePathName = autoName;
    if(!autoName.startsWith("Red_") && !autoName.startsWith("Blu_")){
        alliancePathName = (isRedAlliance ? "Red" : "Blu") + "_" + autoName;
    }
    assert autoPaths.keySet().contains(alliancePathName): "ERROR: no such auto path name found in src/main/deploy/pathplanner/autos: " + alliancePathName;
    System.out.println("About to get Auto Path: " + alliancePathName);
    Command command = driveSubsystem.getAutoPath(alliancePathName);
    assert command != null: "ERROR: unable to get AUTO path for: " + alliancePathName + ".auto";
    return command;
  }


	public void displayEncoders() {
	}

	public void zeroHeading() {
	}


	public void processKeypadCommand(String newKeypadCommand) {
		if(newKeypadCommand.length() > 0){
      System.out.println(newKeypadCommand + "\n");
      if (newKeypadCommand.toLowerCase().contains("cone")){
        // m_ledstring.setBlink(false);
        // m_ledstring.setColor(LedOption.YELLOW);
        // sm_armStateMachine.setGamePiece(GamePiece.CONE);
        System.out.println("\n\nSHOWING YELLOW\n\n");
      }
      else if (newKeypadCommand.toLowerCase().contains("cube")){
        // m_ledstring.setBlink(false);
        // m_ledstring.setColor(LedOption.PURPLE);
        // sm_armStateMachine.setGamePiece(GamePiece.CUBE);
        System.out.println("\n\nSHOWING PURPLE\n\n");
      }
      else if (newKeypadCommand.toLowerCase().contains("clear")){
        // m_ledstring.setBlink(false);
        // m_ledstring.setColor(LedOption.WHITE);
        // sm_armStateMachine.setGamePiece(null);
        System.out.println("\n\nSHOWING WHITE\n\n");
     }
     // delegate to FSM
		 System.out.println("SENDING NEW COMMAND FROM NETWORK TABLES TO FSM: " + newKeypadCommand + "\n\n");
		}
	}
}
