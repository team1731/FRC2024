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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private final Trigger operatorkPOVDown = xboxOperatorController.povDown();
  private final Trigger operatorkPOVUp = xboxOperatorController.povUp();
  private final Trigger operatorkPOVLeft = xboxOperatorController.povLeft();
  private final Trigger operatorkPOVRight = xboxOperatorController.povRight();



  /* Subsystems */
  private CommandSwerveDrivetrain driveSubsystem;
  private VisionSubsystem visionSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private WristSubsystem wristSubsystem;
  private final LEDStringSubsystem m_ledstring;
  private ShooterSubsystem shooterSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private ClimbStateMachine climbStateMachine;
  private IntakeShootStateMachine intakeShootStateMachine;

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
    ElevatorSubsystem s_elevatorSubsystem,
    IntakeShootStateMachine s_intakeShootStateMachine,
    ClimbStateMachine s_climbStateMachine
  ) {

    driveSubsystem = s_driveSubsystem;
    shooterSubsystem = s_shooterSubsystem;
    intakeSubsystem = s_intakeSubsystem;
    wristSubsystem = s_wristSubsystem;
    elevatorSubsystem = s_elevatorSubsystem;
    visionSubsystem = s_visionSubsystem;
    m_ledstring = s_ledstring;
    intakeShootStateMachine = s_intakeShootStateMachine;
    climbStateMachine = s_climbStateMachine;

    if(driveSubsystem.isEnabled()){
      //NamedCommands.registerCommand("RotateLeft", new SequentialCommandGroup(driveSubsystem.rotateRelative(-45.0) ));
      //NamedCommands.registerCommand("RotateRight", new SequentialCommandGroup(driveSubsystem.rotateRelative(-45.0) ));
      NamedCommands.registerCommand("Intake", new SequentialCommandGroup(new IntakeShootStateMachineOneShotCommand(intakeShootStateMachine, ISInput.STOP_SPEAKER),
                                                                         new InstantCommand(() ->  wristSubsystem.moveWrist(0)),
                                                                         new IntakeShootStateMachineOneShotCommand(intakeShootStateMachine, ISInput.START_INTAKE)
                                                                         ));
      NamedCommands.registerCommand("StartShooter", new SequentialCommandGroup(new AutoStartShooter(shooterSubsystem) ));
      NamedCommands.registerCommand("StopShooter", new SequentialCommandGroup(new AutoStopShooter(shooterSubsystem) ));
      NamedCommands.registerCommand("UseVision", new SequentialCommandGroup(new AutoUseVision(intakeSubsystem, wristSubsystem, visionSubsystem)));
      NamedCommands.registerCommand("SetWristB_1_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false,2.93,6.84)) ));
      NamedCommands.registerCommand("SetWristB_1_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.4,false, 3.61, 6.02)) ));
      NamedCommands.registerCommand("SetWristB_1_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.3,false, 3.61, 6.02)) ));
      NamedCommands.registerCommand("SetWristB_2_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(0.0,false,2.64, 5.57)) ));
      NamedCommands.registerCommand("SetWristB_3_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.2,false, 2.65, 4.24)) ));
      NamedCommands.registerCommand("SetWristB_4_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.3,false,2.74, 6.94) )));
      NamedCommands.registerCommand("SetWristB_4_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.6,false, 3.64,6.49))));
      NamedCommands.registerCommand("SetWristB_4_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false, 3.64,6.49)) ));
      NamedCommands.registerCommand("SetWristB_7_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.6,false, 3.03, 3.14)) ));
      NamedCommands.registerCommand("SetWristB_7_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false, 3.03, 3.14)) ));
      NamedCommands.registerCommand("SetWristB_7_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.4,false, 3.03, 3.14)) ));

      NamedCommands.registerCommand("SetWristR_1_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,true,13.57,6.95)) ));  // tuned 319
      NamedCommands.registerCommand("SetWristR_1_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.4,true,12.9,6.02)) ));  // tuned
      NamedCommands.registerCommand("SetWristR_1_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.2,true,12.9,6.02)) ));  // tuned
      NamedCommands.registerCommand("SetWristR_2_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(0.0,true,13.87,5.57)) ));  // tuned
      NamedCommands.registerCommand("SetWristR_3_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.05,true,13.86,4.24)) ));  // tuned
      NamedCommands.registerCommand("SetWristR_4_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.1,true, 13.77, 6.94)) ));
      NamedCommands.registerCommand("SetWristR_4_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.4,true, 12.86,6.49))));
      NamedCommands.registerCommand("SetWristR_4_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.3,true, 12.86,6.496)) ));
      NamedCommands.registerCommand("SetWristR_7_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.4,true,13.67,3.38)) ));
      NamedCommands.registerCommand("SetWristR_7_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,true,13.67,3.38)) ));
      NamedCommands.registerCommand("SetWristr_7_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,true,13.67,3.38)) ));
      NamedCommands.registerCommand("StopVision", new SequentialCommandGroup(new InstantCommand(() -> wristSubsystem.stopMoveWristToTarget())));


      NamedCommands.registerCommand("FireNote", new SequentialCommandGroup(new IntakeShootStateMachineOneShotCommand(intakeShootStateMachine, ISInput.JUST_SHOOT),
                                                                           new InstantCommand(() -> wristSubsystem.stopMoveWristToTarget())));
     // NamedCommands.registerCommand("JustShoot", new SequentialCommandGroup(new IntakeShootStateMachineOneShotCommand(intakeShootStateMachine, ISInput.JUST_SHOOT)));
      NamedCommands.registerCommand("IntakeNoJiggle", new SequentialCommandGroup(new IntakeShootStateMachineOneShotCommand(intakeShootStateMachine, ISInput.STOP_SPEAKER), 
                                                                                 new InstantCommand(() ->  wristSubsystem.moveWrist(0)),
                                                                                 new IntakeShootStateMachineOneShotCommand(intakeShootStateMachine, ISInput.INTAKE_NO_JIGGLE)));
    }
    
    climbStateMachine.setInitialState(CState.ROBOT_LATCHED_ON_CHAIN);

    intakeShootStateMachine.setInitialState(ISState.ALL_STOP);
    
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

 //   ky.whileTrue(driveSubsystem.applyRequest(
 //             () -> driveAtSpeaker.withVelocityX(-(Math.abs(xboxController.getLeftY()) * xboxController.getLeftY()) * MaxSpeed)                                                                                                                     
 //                 .withVelocityY(-(Math.abs(xboxController.getLeftX()) * xboxController.getLeftX()) * MaxSpeed).withTargetDirection(visionSubsystem.getHeadingToSpeakerInRad()) 
 //               
 //         )
 //     );

    ky.whileTrue(new DriveToSpeakerCommand(driveSubsystem, wristSubsystem,visionSubsystem, xboxController));

    //
    //
    //
    // TRADITIONAL WAY
    kLeftTrigger.whileTrue(new IntakeCommand(intakeShootStateMachine, wristSubsystem));   
    //kRightTrigger.whileTrue(new FireNoteSpeakerCommand(intakeSubsystem, shooterSubsystem));
    //
    //
    // STATE MACHINE WAY
   //  kLeftTrigger.onTrue(new IntakeShootStateMachineCommand(intakeShootStateMachine, ISInput.START_INTAKE))
   //              .onFalse(new IntakeShootStateMachineCommand(intakeShootStateMachine, ISInput.STOP_INTAKE));
     kRightTrigger.whileTrue(new IntakeShootStateMachineCommand(intakeShootStateMachine, ISInput.START_SPEAKER))
                  .onFalse(new IntakeShootStateMachineOneShotCommand(intakeShootStateMachine, ISInput.STOP_SPEAKER));
    //
    //
    //

   // kRightBumper.whileTrue(new AmpScoringCommand(intakeSubsystem, elevatorSubsystem, wristSubsystem)));
    kRightBumper.onTrue(new AmpScoringReverseCommand(intakeSubsystem, elevatorSubsystem, wristSubsystem))
                .onFalse(new ScoreAmpAndRetractReverseCommand(intakeShootStateMachine, elevatorSubsystem, wristSubsystem));
    kLeftBumper.whileTrue(new ClimbCommand(intakeSubsystem, shooterSubsystem, elevatorSubsystem, wristSubsystem));
    //kx.whileTrue(new TrapScoringCommand(intakeSubsystem, elevatorSubsystem, wristSubsystem));
    kx.whileTrue(new ClimbWithStateMachine(climbStateMachine));

    ka.onTrue(new InstantCommand(() -> wristSubsystem.retractTrapFlap()));
    kb.onTrue(new InstantCommand(() -> wristSubsystem.extendTrapFlap()));
 //  Comment out the two lines above and uncomment this to tune shooting angles
   //  Also uncomment the call to get the distance to the speaker in the period of vision subsystem (that sends the data to smartdashbord among other things)
   //     ka.onTrue(new InstantCommand(() -> wristSubsystem.jogDown()))
    //    .onFalse(new InstantCommand(() -> wristSubsystem.stopJog()));

   //     kb.onTrue(new InstantCommand(() -> wristSubsystem.jogUp()))
   //     .onFalse(new InstantCommand(() -> wristSubsystem.stopJog()));

 

    kStart.onTrue(driveSubsystem.runOnce(() -> driveSubsystem.seedFieldRelative()));
    
    operatorkLeftBumper.onTrue(new InstantCommand(() -> {
      shooterSubsystem.stopShooting();
    }));
    operatorkRightBumper.onTrue(new InstantCommand(() -> {
      shooterSubsystem.shoot();
    }));

    operatorkPOVDown.onTrue(new InstantCommand(() -> {
      wristSubsystem.fudgeUp();
    }));
    operatorkPOVUp.onTrue(new InstantCommand(() -> {
      wristSubsystem.fudgeDown();
    }));

    operatorkPOVLeft.onTrue(new InstantCommand(() -> {
      visionSubsystem.shootOnMoveFudgeDown();
    }));
    operatorkPOVRight.onTrue(new InstantCommand(() -> {
      visionSubsystem.shootOnMoveFudgeUp();
    }));
    /* 
    operatorkStart
        .onTrue(new InstantCommand(() -> intakeSubsystem.reverseIntake()))
        .onFalse(new InstantCommand(() -> intakeSubsystem.stopReverseIntake()));
*/

    operatorkStart.whileTrue(new IntakeShootStateMachineCommand(intakeShootStateMachine, ISInput.START_EJECT))
                 .onFalse(new IntakeShootStateMachineOneShotCommand(intakeShootStateMachine, ISInput.STOP_EJECT));

    // Far Shot
    operatorky.onTrue(new InstantCommand(() -> wristSubsystem.moveWrist(12)))  // this is now over the stage
        .onFalse(new InstantCommand(() -> wristSubsystem.moveWrist(0)));
    // Safe Shot
    operatorkb.onTrue(new InstantCommand(() -> wristSubsystem.moveWrist(22*0.6)))
        .onFalse(new InstantCommand(() -> wristSubsystem.moveWrist(0)));
    // Line Shot
    operatorka.onTrue(new InstantCommand(() -> wristSubsystem.moveWrist(15*0.6)))
        .onFalse(new InstantCommand(() -> wristSubsystem.moveWrist(0)));
 //   operatorkRightTrigger.onTrue(new JiggleCommand(intakeShootSubsystem, shooterSubsystem));

    operatorkRightTrigger.whileTrue(new IntakeShootStateMachineCommand(intakeShootStateMachine, ISInput.START_JIGGLE));
                
    operatorkLeftTrigger.whileTrue(new IntakeShootStateMachineCommand(intakeShootStateMachine, ISInput.START_SHOOT_INTAKE))
      .onFalse(new IntakeShootStateMachineOneShotCommand(intakeShootStateMachine, ISInput.STOP_SHOOT_INTAKE));

    operatorkx.onTrue(new InstantCommand(() -> wristSubsystem.slowlyDown()))
        .onFalse(new InstantCommand(() -> wristSubsystem.stop()));

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
    //System.out.println("About to get Auto Path: " + alliancePathName);
    Command command = driveSubsystem.getAutoPath(alliancePathName);
    assert command != null: "ERROR: unable to get AUTO path for: " + alliancePathName + ".auto";
    System.out.println("\nAUTO CODE being used by the software --> " + alliancePathName + ", RED/BLUE flipping is " + (flipRedBlue ? "ON" : "OFF") + "\n");
    SmartDashboard.putString("AUTO_FILE_IN_USE", alliancePathName);
    SmartDashboard.putBoolean("RED_BLUE_FLIPPING", flipRedBlue);
    return command;
  }


}
