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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.OperatorConsoleConstants;
import frc.robot.Constants.GamePiece;


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
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  /* Controllers */
  private final CommandXboxController xboxController = new CommandXboxController(0);
  private final Joystick operator = new Joystick(1);


  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final Trigger kStart = xboxController.start();
  private final Trigger ky = xboxController.y();
  private final Trigger kb = xboxController.b();
  private final Trigger ka = xboxController.a();
  private final Trigger kx = xboxController.x();
  private final Trigger kLeftBumper = xboxController.leftBumper();
  private final Trigger kRightBumper = xboxController.rightBumper();
  private final Trigger kLeftTrigger = xboxController.leftTrigger();
  private final Trigger kRightTrigger = xboxController.rightTrigger();

  /* Operator Buttons */
  private final JoystickButton kTurnForwardBtn = new JoystickButton(operator,OperatorConsoleConstants.kPreventScoreBtnId);
  private final JoystickButton kWheelLockBtn = new JoystickButton(operator,OperatorConsoleConstants.kWheelLockBtnId);
  private final JoystickButton kTurnBackBtn = new JoystickButton(operator,OperatorConsoleConstants.kReleaseBtnId);
  private final JoystickButton kIntakeBtn = new JoystickButton(operator,OperatorConsoleConstants.kIntakeBtnId);
  private final JoystickButton kxINT = new JoystickButton(operator, OperatorConsoleConstants.kxAxis);
  // Operator switches
  private final JoystickButton kKillSwitch = new JoystickButton(operator,OperatorConsoleConstants.kKillSwitchId);
  private final JoystickButton kAutoRecoverySwitch = new JoystickButton(operator,OperatorConsoleConstants.kAutoRecoverySwitchId);
  private final JoystickButton kShooterSwitch = new JoystickButton(operator,OperatorConsoleConstants.kShooterSwitch);
  private final JoystickButton kAngleSwitch = new JoystickButton(operator,OperatorConsoleConstants.kScoreHighSwitchId);
  private final JoystickButton kMediumScoreSwitch = new JoystickButton(operator,OperatorConsoleConstants.kScoreMediumSwitchId);
  private final JoystickButton kWristAngle3 = new JoystickButton(operator,OperatorConsoleConstants.kWristAngle3);
  private final JoystickButton kWristAngle4 = new JoystickButton(operator,OperatorConsoleConstants.kWristAngle4);
  private final JoystickButton kWristSwitch = new JoystickButton(operator,OperatorConsoleConstants.kWristSwitchId);
  

  // Operator sticks
  public final int kDistalAxis = OperatorConsoleConstants.kDistalAxisId;
  public final int kProximalAxis = OperatorConsoleConstants.kProximalAxisId;


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

  private GamePiece storedPiece; // used to temporarily store game piece setting when using cone flip feature

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
      NamedCommands.registerCommand("SetWristNote1", new SequentialCommandGroup(new InstantCommand(() ->  s_wristSubsystem.moveWrist(22)) ));
      NamedCommands.registerCommand("SetWristNote2", new SequentialCommandGroup(new InstantCommand(() ->  s_wristSubsystem.moveWrist(22)) ));
      NamedCommands.registerCommand("SetWristNote3", new SequentialCommandGroup(new InstantCommand(() ->  s_wristSubsystem.moveWrist(22)) ));
      NamedCommands.registerCommand("SetWristLongShot", new SequentialCommandGroup(new InstantCommand(() ->  s_wristSubsystem.moveWrist(22)) ));
      NamedCommands.registerCommand("SetWristLineShot", new SequentialCommandGroup(new InstantCommand(() ->  s_wristSubsystem.moveWrist(22)) ));
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
      driveSubsystem.applyRequest(() -> drive.withVelocityX(-(Math.abs(xboxController.getLeftY()) * xboxController.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(-(Math.abs(xboxController.getLeftX()) * xboxController.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(-xboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
      ));    
    }
    
    // RESET BUTTON
    // kStart.onTrue(new InstantCommand(() -> {
    //   s_Swerve.zeroGyro();
    //   s_Swerve.adjustWheelEncoders(); 
    //   //s_armSubSystem.resetArmEncoders();
    // }));

    kLeftTrigger.whileTrue(new IntakeCommand(s_intakeSubsystem, s_wristSubsystem));
    kRightTrigger.whileTrue(new FireNoteSpeakerCommand(s_intakeSubsystem, s_ShooterSubsystem));
  
    kRightBumper.whileTrue(new AmpScoringCommand(s_intakeSubsystem, elevatorSubsystem, s_wristSubsystem));
    kLeftBumper.whileTrue(new ClimbCommand(s_intakeSubsystem, s_ShooterSubsystem, elevatorSubsystem, s_wristSubsystem));

    kShooterSwitch.onTrue(new InstantCommand(() -> {
        s_ShooterSubsystem.shoot();
    }))
    .onFalse(new InstantCommand(() -> {
        s_ShooterSubsystem.stopShooting();
    }));


    

    // REVERSE FEEDER INTAKE -- Temporarily commented out for manual wrist adjustments
    // ka
    //   .onTrue(new InstantCommand(() -> s_intakeSubsystem.reverseIntake()))
    //   .onFalse(new InstantCommand(() -> s_intakeSubsystem.stopIntake()));
  
    // kb
    //   .onTrue(new InstantCommand(() -> s_intakeSubsystem.reverseFeed()))
    //   .onFalse(new InstantCommand(() -> s_intakeSubsystem.stopFeed()));
    kWheelLockBtn
       .onTrue(new InstantCommand(() -> s_intakeSubsystem.reverseFeed()))
       .onFalse(new InstantCommand(() -> s_intakeSubsystem.stopFeed()));
    kx.whileTrue(new TrapScoringCommand(s_intakeSubsystem, elevatorSubsystem, s_wristSubsystem));

    ka.onTrue(new InstantCommand(() -> s_wristSubsystem.retractTrapFlap()));
    kb.onTrue(new InstantCommand(() -> s_wristSubsystem.extendTrapFlap()));

    
    kStart.onTrue(driveSubsystem.runOnce(() -> driveSubsystem.seedFieldRelative()));
    // // ELEVATOR - EXTEND AND HOME
    // kx
    //   .onTrue(new InstantCommand(() -> elevatorSubsystem.sendElevatorHome()))
    //   .onFalse(new InstantCommand(() -> elevatorSubsystem.getElevatorPosition()));

    // ky
    //   .onTrue(new InstantCommand(() -> elevatorSubsystem.sendElevatorHome()))
    //   .onFalse(new InstantCommand(() -> elevatorSubsystem.getElevatorPosition()));
    //kx
    //  .onTrue(new InstantCommand(() -> elevatorSubsystem.liftElevator()))
    //  .onFalse(new InstantCommand(() -> elevatorSubsystem.stopElevator()));

   // ky
    //  .onTrue(new InstantCommand(() -> elevatorSubsystem.lowerElevator()))
    //  .onFalse(new InstantCommand(() -> elevatorSubsystem.stopElevator()));


    // WRIST - FORWARD AND BACK
    //ka
    //  .onTrue(new InstantCommand(() -> s_wristSubsystem.turnWristForward()))
    //  .onFalse(new InstantCommand(() -> s_wristSubsystem.stopWrist()));
    
    //kb
    //  .onTrue(new InstantCommand(() -> s_wristSubsystem.turnWristBack()))
    //  .onFalse(new InstantCommand(() -> s_wristSubsystem.stopWrist()));
    
    // SCORE HIGH/MED/LOW BUTTONS
    // ky.whileTrue((new ArmScoreCommand(sm_armStateMachine, ArmSequence.SCORE_HIGH, operator, kDistalAxis)));
    // kb.whileTrue((new ArmScoreCommand(sm_armStateMachine, ArmSequence.SCORE_MEDIUM, operator, kDistalAxis)));
    // ka.whileTrue((new ArmScoreCommand(sm_armStateMachine, ArmSequence.SCORE_LOW, operator, kDistalAxis)));

    // CLEAR/RESET PATH BUTTON
    //kx.whileTrue(new InstantCommand(() -> sm_armStateMachine.clearCurrentPath()));

    // BUMPERS - EITHER START/STOP RECORDING  - OR -  PICKUP HIGH/RUN OPERATOR SEQUENCE
    // if(LogWriter.isArmRecordingEnabled()) {
    //   kLeftBumper.onTrue(new InstantCommand(() -> s_armSubSystem.startRecordingArmPath()));
    //   kRightBumper.onTrue(new InstantCommand(() -> s_armSubSystem.stopRecordingArmPath()));
    // } else {
    //   kLeftBumper.whileTrue(new ArmPickupCommand(sm_armStateMachine, ArmSequence.PICKUP_HIGH, operator, kDistalAxis));
    //   kRightBumper.whileTrue(new ArmScoreCommand(sm_armStateMachine, ArmSequence.READ_OPERATOR_ENTRY, operator, kDistalAxis));
    // }

    // TRIGGERS - PICKUP LOW AND PICKUP DOWNED CONE
    // kLeftTrigger.whileTrue(new ArmPickupCommand(sm_armStateMachine, ArmSequence.PICKUP_LOW, operator, kDistalAxis));

    // kRightBumper.onTrue(new InstantCommand(() -> s_intakeSubsystem.intake()));
    // kRightBumper.onFalse(new InstantCommand(() -> s_intakeSubsystem.stopIntake()));
       
    /*
     * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
     * OPERATOR BUTTONS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    */

    // WRISt MODE ON/OFF SWITCH
    //kWristSwitch.onTrue(new InstantCommand(() -> {
     // System.out.println("RobotContainer: Wrist to Extended.");
     // m_ledstring.setBlink(false);
     // m_ledstring.setColor(LedOption.YELLOW);
     // s_wristSubsystem.wristExtended();
    //}));

    //kWristSwitch.onFalse(new InstantCommand(() -> {
    //  System.out.println("RobotContainer: Wrist to Home.");
    //  m_ledstring.setBlink(false);
    //  m_ledstring.setColor(LedOption.PURPLE);
    //  s_wristSubsystem.wristHome();
    //}));

    driveSubsystem.registerTelemetry(logger::telemeterize);


    // PREVENT SCORE
    // kPreventScoreBtn.whileTrue(new InstantCommand(() -> sm_armStateMachine.setAllowScore(false)));
    // kPreventScoreBtn.whileFalse(new InstantCommand(() -> sm_armStateMachine.setAllowScore(true)));

    // LOCK WHEELS IN X-PATTERN
    // kWheelLockBtn.whileTrue(new InstantCommand(() -> s_Swerve.setLockWheels(true)));
    // kWheelLockBtn.whileFalse(new InstantCommand(() -> s_Swerve.setLockWheels(false)));

    // ON REQUEST INTAKE OR EJECT BUTTONS
    // kIntakeBtn.whileTrue(new InstantCommand(() -> sm_armStateMachine.intake()));
    // kIntakeBtn.whileFalse(new InstantCommand(() -> sm_armStateMachine.releaseIntake()));
    // kReleaseBtn.whileTrue(new InstantCommand(() -> sm_armStateMachine.release()));
    // kReleaseBtn.whileFalse(new InstantCommand(() -> sm_armStateMachine.stopRelease()));

    // EMERENCY MODE AND AUTO-RECOVERY SWITCH
    // kKillSwitch.onTrue(new InstantCommand(() -> {
    //   sm_armStateMachine.addJoystickControl(operator, kProximalAxis, false);
    //   sm_armStateMachine.addJoystickControl(operator, kDistalAxis, false);
    //   sm_armStateMachine.emergencyInterrupt();
    // }));
    // kAutoRecoverySwitch.onTrue(new InstantCommand(() -> sm_armStateMachine.attemptAutoRecovery()));

    // // HIGH PICKUP SWITCH - FEEDER OR SHELF
    // kHighPickupSwitch.onTrue(new InstantCommand(() -> {
    //   sm_armStateMachine.setHighPickup(HighPickup.FEEDER);
    // }));
    // kHighPickupSwitch.onFalse(new InstantCommand(() -> {
    //   sm_armStateMachine.setHighPickup(HighPickup.SHELF);
    // }));

    // // HIGH/MED/LOW SCORE PRE-LOAD SWITCH
     kAngleSwitch.onTrue(new InstantCommand(() -> s_wristSubsystem.moveWrist(15)));
     kMediumScoreSwitch.onTrue(new InstantCommand(() ->  s_wristSubsystem.moveWrist(0)));

    // // THIEF MODE ON/OFF SWITCH
   // kThiefOnSwitch.onTrue(new InstantCommand(() -> sm_armStateMachine.setIsInThiefMode(true)));
    // kThiefOffSwitch.onTrue(new InstantCommand(() -> sm_armStateMachine.setIsInThiefMode(false)));
    kWristAngle3.onTrue(new InstantCommand(() ->  s_wristSubsystem.moveWrist(22)));
    kWristAngle4.onTrue(new InstantCommand(() ->  s_wristSubsystem.moveWrist(30)));
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
