// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.CommandSwerveDrivetrain;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.TunerConstants;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.util.log.LogWriter;
import frc.robot.Constants.OperatorConsoleConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.HighPickup;
import frc.robot.Constants.OpConstants.LedOption;
import frc.robot.TunerConstants.*;


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
  private final JoystickButton kPreventScoreBtn = new JoystickButton(operator,OperatorConsoleConstants.kPreventScoreBtnId);
  private final JoystickButton kWheelLockBtn = new JoystickButton(operator,OperatorConsoleConstants.kWheelLockBtnId);
  private final JoystickButton kReleaseBtn = new JoystickButton(operator,OperatorConsoleConstants.kReleaseBtnId);
  private final JoystickButton kIntakeBtn = new JoystickButton(operator,OperatorConsoleConstants.kIntakeBtnId);
  private final JoystickButton kxINT = new JoystickButton(operator, OperatorConsoleConstants.kxAxis);
  // Operator switches
  private final JoystickButton kKillSwitch = new JoystickButton(operator,OperatorConsoleConstants.kKillSwitchId);
  private final JoystickButton kAutoRecoverySwitch = new JoystickButton(operator,OperatorConsoleConstants.kAutoRecoverySwitchId);
  private final JoystickButton kShooterSwitch = new JoystickButton(operator,OperatorConsoleConstants.kShooterSwitch);
  private final JoystickButton kHighScoreSwitch = new JoystickButton(operator,OperatorConsoleConstants.kScoreHighSwitchId);
  private final JoystickButton kMediumScoreSwitch = new JoystickButton(operator,OperatorConsoleConstants.kScoreMediumSwitchId);
  // private final JoystickButton kThiefOnSwitch = new JoystickButton(operator,OperatorConsoleConstants.kThiefOnSwitchId);
  // private final JoystickButton kThiefOffSwitch = new JoystickButton(operator,OperatorConsoleConstants.kThiefOffSwitchId);
  private final JoystickButton kWristSwitch = new JoystickButton(operator,OperatorConsoleConstants.kWristSwitchId);
  

  // Operator sticks
  public final int kDistalAxis = OperatorConsoleConstants.kDistalAxisId;
  public final int kProximalAxis = OperatorConsoleConstants.kProximalAxisId;


  /* Subsystems */
  private CommandSwerveDrivetrain driveSubsystem;
  // private PoseEstimatorSubsystem s_poseEstimatorSubsystem;
  private IntakeSubsystem s_intakeSubsystem;
  private WristSubsystem s_wristSubsystem;
  private final LEDStringSubsystem m_ledstring;
  private ShooterSubsystem s_ShooterSubsystem;
  private ElevatorSubsystem elevatorSubsystem;

  /* Auto Paths */
  private static List<String> autoPaths;

  private GamePiece storedPiece; // used to temporarily store game piece setting when using cone flip feature

  // The container for the robot. Contains subsystems, OI devices, and commands. 
  public RobotContainer(
          CommandSwerveDrivetrain driveSubsystem,
          ShooterSubsystem ShooterSubsystem,
          // PoseEstimatorSubsystem poseEstimatorSubsystem,
          IntakeSubsystem intakeSubsystem,
          WristSubsystem wristSubsystem,
          LEDStringSubsystem m_ledstring,
          ElevatorSubsystem elevatorSubsystem
          ) {
    
	  boolean fieldRelative = true;
    boolean openLoop = false;
    this.driveSubsystem = driveSubsystem;
    s_ShooterSubsystem = ShooterSubsystem;
    s_intakeSubsystem = intakeSubsystem;
    s_wristSubsystem = wristSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    // s_poseEstimatorSubsystem = poseEstimatorSubsystem;
     //sm_armStateMachine = armSubsystem.getStateMachine();

    this.m_ledstring = m_ledstring;

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
    driveSubsystem.setDefaultCommand( // Drivetrain will execute this command periodically
    driveSubsystem.applyRequest(() -> drive.withVelocityX(-xboxController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(-xboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-xboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));    

    // RESET BUTTON
    // kStart.onTrue(new InstantCommand(() -> {
    //   s_Swerve.zeroGyro();
    //   s_Swerve.adjustWheelEncoders(); 
    //   //s_armSubSystem.resetArmEncoders();
    // }));

    // kLeftTrigger.whileTrue(new IntakeCommand(s_intakeSubsystem, s_ShooterSubsystem, s_poseEstimatorSubsystem));
    // kRightTrigger.whileTrue(new FireNoteSpeakerCommand(s_intakeSubsystem, s_ShooterSubsystem, s_poseEstimatorSubsystem));
  
    kRightBumper.whileTrue(new AmpScoringCommand(s_intakeSubsystem, elevatorSubsystem, s_wristSubsystem));
    kLeftBumper.whileTrue(new ClimbCommand(s_intakeSubsystem, s_ShooterSubsystem, elevatorSubsystem, s_wristSubsystem));

    kShooterSwitch.onTrue(new InstantCommand(() -> {
        s_ShooterSubsystem.shoot();
    }))
    .onFalse(new InstantCommand(() -> {
        s_ShooterSubsystem.stopShooting();
    }));


  

    // REVERSE FEEDER INTAKE
    ka
      .onTrue(new InstantCommand(() -> s_intakeSubsystem.reverseIntake()))
      .onFalse(new InstantCommand(() -> s_intakeSubsystem.stopIntake()));
  
    kb
      .onTrue(new InstantCommand(() -> s_intakeSubsystem.reverseFeed()))
      .onFalse(new InstantCommand(() -> s_intakeSubsystem.stopFeed()));

    // ELEVATOR - EXTEND AND HOME
    kx
      .onTrue(new InstantCommand(() -> elevatorSubsystem.sendElevatorUp()))
      .onFalse(new InstantCommand(() -> elevatorSubsystem.getElevatorPosition()));

    ky
      .onTrue(new InstantCommand(() -> elevatorSubsystem.sendElevatorHome()))
      .onFalse(new InstantCommand(() -> elevatorSubsystem.getElevatorPosition()));
    
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
    kWristSwitch.onTrue(new InstantCommand(() -> {
      System.out.println("RobotContainer: Wrist to Extended.");
      m_ledstring.setBlink(false);
      m_ledstring.setColor(LedOption.YELLOW);
      s_wristSubsystem.wristExtended();
    }));

    kWristSwitch.onFalse(new InstantCommand(() -> {
      System.out.println("RobotContainer: Wrist to Home.");
      m_ledstring.setBlink(false);
      m_ledstring.setColor(LedOption.PURPLE);
      s_wristSubsystem.wristHome();
    }));

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
    // kHighScoreSwitch.onTrue(new InstantCommand(() -> sm_armStateMachine.setOperatorSequence(OperatorConsoleConstants.kScoreHighSwitchId)));
    // kMediumScoreSwitch.onTrue(new InstantCommand(() -> sm_armStateMachine.setOperatorSequence(OperatorConsoleConstants.kScoreMediumSwitchId)));

    // // THIEF MODE ON/OFF SWITCH
    // kThiefOnSwitch.onTrue(new InstantCommand(() -> sm_armStateMachine.setIsInThiefMode(true)));
    // kThiefOffSwitch.onTrue(new InstantCommand(() -> sm_armStateMachine.setIsInThiefMode(false)));
  }

  public static String[] deriveAutoModes() {
    List<String> autoModes = new ArrayList<String>();
    List<String> autoNames = findPaths(new File(Filesystem.getLaunchDirectory(), (Robot.isReal() ? "home/lvuser" : "src/main") + "/deploy/pathplanner/autos"));
    for(String autoName : autoNames){
      if(!autoModes.contains(autoName)){
        autoModes.add(autoName);
      }
    }
    autoModes.sort((p1, p2) -> p1.compareTo(p2));
    return autoModes.toArray(String[]::new);
  }

  private static List<String> findPaths(File directory){
    List<String> autoNames = AutoBuilder.getAllAutoNames();
    // List<String> paths = new ArrayList<String>();
    // if(!directory.exists()){
    //   System.out.println("FATAL: path directory not found! " + directory.getAbsolutePath());
    // }
    // else {
    //   File[] files = directory.listFiles();
    //   if(files == null){
    //     System.out.println("FATAL: I/O error or NOT a directory: " + directory);
    //   }
    //   else
    //   {
    //     for (File file : files) {
    //         String fileName = file.getName();
    //         if ((fileName.startsWith("Blu") || fileName.startsWith("Red")) && fileName.endsWith(".auto")) {
    //           System.out.println(file.getAbsolutePath());
    //           if(!paths.contains(fileName)){
    //             paths.add(fileName);
    //           }
    //         }
    //     }
    //   }
    // }
    return autoNames;
  }

  // public Command getNamedAutonomousCommand(String autoName, boolean isRedAlliance) {
  //   String alliancePathName = (isRedAlliance ? "Red" : "Blu") + "_" + autoName;
  //   assert autoPaths.contains(alliancePathName): "ERROR: no such auto path name found in src/main/deploy/pathplanner/autos: " + alliancePathName;
  //   double maxVelocity     = 4.0;
  //   double maxAcceleration = 2.0;
  //   switch(alliancePathName){
  //     case "_1_Charger_Mid_1pc_Blue":    maxVelocity = 4.0; maxAcceleration = 1.8; break;
  //     case "_1_Charger_Mid_1pc_Red":     maxVelocity = 4.0; maxAcceleration = 1.8; break;
  //     case "_2_Feeder_3pc_Blue":         maxVelocity = 3.0; maxAcceleration = 1.8; break;
  //     case "_2_Feeder_3pc_Red":          maxVelocity = 3.0; maxAcceleration = 1.8; break;
  //     case "_3_Cable_3pc_Blue":          maxVelocity = 4.0; maxAcceleration = 2.0; break;
  //     case "_3_Cable_3pc_Red":           maxVelocity = 4.0; maxAcceleration = 2.0; break;
  //     case "_4_Feeder_2pc_Charger_Blue": maxVelocity = 4.0; maxAcceleration = 2.0; break;
  //     case "_4_Feeder_2pc_Charger_Red":  maxVelocity = 4.0; maxAcceleration = 2.0; break;
  //     case "_5_Charger_Mid_2pc_Blue":    maxVelocity = 4.0; maxAcceleration = 1.8; break;
  //     case "_5_Charger_Mid_2pc_Red":     maxVelocity = 4.0; maxAcceleration = 1.8; break;
  //     case "_6_Feeder_real3pc_Blue":     maxVelocity = 4.0; maxAcceleration = 2.1; break;
  //     case "_6_Feeder_real3pc_Red":      maxVelocity = 4.0; maxAcceleration = 2.1; break;
  //     case "_7_Charger_Mid_2pc_Blue":    maxVelocity = 4.2; maxAcceleration = 2.3; break;
  //     case "_7_Charger_Mid_2pc_Red":     maxVelocity = 4.2; maxAcceleration = 2.3; break;
  //     case "_8_Cable_real3pc_Red":       maxVelocity = 4.0; maxAcceleration = 2.1; break;
  //     case "_8_Cable_real3pc_Blue":      maxVelocity = 4.0; maxAcceleration = 2.1; break;
      

  //     default: System.out.println("WARNING: USING DEFAULT MAX VELOCITY AND MAX ACCELERATION FOR AUTO MODE: " + alliancePathName);
  //   }
  //   return new PathPlannerCommandGroup(alliancePathName, s_Swerve, s_poseEstimatorSubsystem, maxVelocity, maxAcceleration);
  // }


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

  private boolean isRedAlliance(){
    return DriverStation.getAlliance().equals(DriverStation.Alliance.Red);
    }
}
