// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private VisionSubsystem visionSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private WristSubsystem wristSubsystem;
  private final LEDStringSubsystem m_ledstring;
  private ShooterSubsystem shooterSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ClimbStateMachine climbStateMachine;

  /* Auto Paths */
  private static HashMap<String, String> autoPaths;

  private static boolean flipRedBlue;

  public static boolean isFlipRedBlue(){
    return flipRedBlue;
  }



  // The container for the robot. Contains subsystems, OI devices, and commands. 
  public RobotContainer(
    CommandSwerveDrivetrain s_driveSubsystem,
    ShooterSubsystem s_shooterSubsystem,
    VisionSubsystem s_visionSubsystem,
    IntakeSubsystem s_intakeSubsystem,
    WristSubsystem s_wristSubsystem,
    LEDStringSubsystem s_ledstring,
    ElevatorSubsystem s_elevatorSubsystem
  ) {

    driveSubsystem = s_driveSubsystem;
    shooterSubsystem = s_shooterSubsystem;
    intakeSubsystem = s_intakeSubsystem;
    wristSubsystem = s_wristSubsystem;
    elevatorSubsystem = s_elevatorSubsystem;
    visionSubsystem = s_visionSubsystem;
    m_ledstring = s_ledstring;

    if(driveSubsystem.isEnabled()){
      //NamedCommands.registerCommand("RotateLeft", new SequentialCommandGroup(driveSubsystem.rotateRelative(-45.0) ));
      //NamedCommands.registerCommand("RotateRight", new SequentialCommandGroup(driveSubsystem.rotateRelative(-45.0) ));
      NamedCommands.registerCommand("Intake", new SequentialCommandGroup(new AutoIntake(intakeSubsystem, wristSubsystem) ));
      NamedCommands.registerCommand("StartShooter", new SequentialCommandGroup(new AutoStartShooter(shooterSubsystem) ));
      NamedCommands.registerCommand("StopShooter", new SequentialCommandGroup(new AutoStopShooter(shooterSubsystem) ));
      NamedCommands.registerCommand("SetWristNote1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWrist(24)) ));
      NamedCommands.registerCommand("SetWristNote1Red", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWrist(25)) ));
      NamedCommands.registerCommand("SetWristNote2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWrist(19)) ));
      NamedCommands.registerCommand("SetWristNote2Red", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWrist(21)) ));
      NamedCommands.registerCommand("SetWristNote3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWrist(18.5)) ));
      NamedCommands.registerCommand("SetWristNote3Red", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWrist(23)) ));
      NamedCommands.registerCommand("SetWristLongShot", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWrist(23)) ));
      NamedCommands.registerCommand("SetWristLongShotA", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWrist(25)) ));
      NamedCommands.registerCommand("SetWristLineShot", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWrist(10)) ));
      NamedCommands.registerCommand("SetWristBlu7Long", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWrist(28)) ));
      NamedCommands.registerCommand("SetWristRed7Long", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWrist(27)) ));
      NamedCommands.registerCommand("FireNote", new SequentialCommandGroup(new AutoFireNote( intakeSubsystem, shooterSubsystem) ));
    }

    climbStateMachine = new ClimbStateMachine(intakeSubsystem, shooterSubsystem, elevatorSubsystem, wristSubsystem);
    climbStateMachine.setInitialState(State.ROBOT_LATCHED_ON_CHAIN);

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
          )
      );
    }

    kLeftTrigger.whileTrue(new IntakeCommand(intakeSubsystem, wristSubsystem));
                


    kRightTrigger.whileTrue(new FireNoteSpeakerCommand(intakeSubsystem, shooterSubsystem));
   // kRightBumper.whileTrue(new AmpScoringCommand(intakeSubsystem, elevatorSubsystem, wristSubsystem)));
    kRightBumper.onTrue(new AmpScoringReverseCommand(intakeSubsystem, elevatorSubsystem, wristSubsystem))
                .onFalse(new ScoreAmpAndRetractReverseCommand(shooterSubsystem, intakeSubsystem, elevatorSubsystem, wristSubsystem));
    kLeftBumper.whileTrue(new ClimbCommand(intakeSubsystem, shooterSubsystem, elevatorSubsystem, wristSubsystem));
    //kx.whileTrue(new TrapScoringCommand(intakeSubsystem, elevatorSubsystem, wristSubsystem));
    kx.whileTrue(new ClimbWithStateMachine(climbStateMachine));

    ka.onTrue(new InstantCommand(() -> wristSubsystem.retractTrapFlap()));
    kb.onTrue(new InstantCommand(() -> wristSubsystem.extendTrapFlap()));

    kStart.onTrue(driveSubsystem.runOnce(() -> driveSubsystem.seedFieldRelative()));

    operatorkLeftBumper.onTrue(new InstantCommand(() -> {
      shooterSubsystem.shoot();
    }));
    operatorkRightBumper.onTrue(new InstantCommand(() -> {
      shooterSubsystem.stopShooting();
    }));
    operatorkStart
        .onTrue(new InstantCommand(() -> intakeSubsystem.reverseIntake()))
        .onFalse(new InstantCommand(() -> intakeSubsystem.stopReverseIntake()));

    // Far Shot
    operatorky.onTrue(new InstantCommand(() -> wristSubsystem.moveWrist(25)))
        .onFalse(new InstantCommand(() -> wristSubsystem.moveWrist(0)));
    // Safe Shot
    operatorkb.onTrue(new InstantCommand(() -> wristSubsystem.moveWrist(22)))
        .onFalse(new InstantCommand(() -> wristSubsystem.moveWrist(0)));
    // Line Shot
    operatorka.onTrue(new InstantCommand(() -> wristSubsystem.moveWrist(15)))
        .onFalse(new InstantCommand(() -> wristSubsystem.moveWrist(0)));
    operatorkRightTrigger.whileTrue(new JiggleCommand(intakeSubsystem, shooterSubsystem));

    driveSubsystem.registerTelemetry(logger::telemeterize);
  }

  public static String[] deriveAutoModes() {
    autoPaths = findPaths(new File(Filesystem.getLaunchDirectory(), (Robot.isReal() ? "home/lvuser" : "src/main") + "/deploy/pathplanner/autos"));
    List<String> autoModes = new ArrayList<String>();
    for(String key : autoPaths.keySet()){
      String stripKey = key.toString();
      if(stripKey.startsWith("Red_") || stripKey.startsWith("Blu_")){
        stripKey = stripKey.substring(4, stripKey.length());
      }
      if(!autoModes.contains(stripKey)){
        autoModes.add(stripKey);
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
    // if the named auto (red or blue) exists, use it as-is and do NOT flip the field (red/blue)
    if(autoPaths.keySet().contains(alliancePathName)){
      flipRedBlue = false;
    }
    // if the named auto does not exist (so there isn't a red one), use the blue one and flip the field
    else if(isRedAlliance && alliancePathName.startsWith("Red_")) {
      alliancePathName = alliancePathName.replace("Red_", "Blu_");
      assert autoPaths.keySet().contains(alliancePathName): "ERROR: you need to create " + alliancePathName;
      flipRedBlue = true;
    }
    else {
      System.out.println("ERROR: no such auto path name found in src/main/deploy/pathplanner/autos: " + alliancePathName);
    }
    System.out.println("About to get Auto Path: " + alliancePathName);
    Command command = driveSubsystem.getAutoPath(alliancePathName);
    assert command != null: "ERROR: unable to get AUTO path for: " + alliancePathName + ".auto";
    System.out.println("\nAUTO CODE being used by the software --> " + alliancePathName + ", RED/BLUE flipping is " + (flipRedBlue ? "ON" : "OFF") + "\n");
    SmartDashboard.putString("AUTO_FILE_IN_USE", alliancePathName);
    SmartDashboard.putBoolean("RED_BLUE_FLIPPING", flipRedBlue);
    return command;
  }


}
