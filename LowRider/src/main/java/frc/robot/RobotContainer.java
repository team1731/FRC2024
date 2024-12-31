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
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private double MaxAngularRate = 2.0 * Math.PI; // 3/4 of a rotation per second max angular velocity

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
  private final Trigger kPOVUp = xboxController.povUp();

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
 // private Command Blu_10_Command;
 // private Command Red_10_Command;

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
      NamedCommands.registerCommand("SetWristB_1_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.6,false, 3.61, 6.02)) ));
      NamedCommands.registerCommand("SetWristB_1_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.7,false, 3.61, 6.02)) ));

      NamedCommands.registerCommand("SetWristB_1B_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false,2.93,6.84)) ));
      NamedCommands.registerCommand("SetWristB_1B_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.8,false, 3.61, 6.02)) ));
      NamedCommands.registerCommand("SetWristB_1B_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-1.5,false, 3.61, 6.02)) ));

      NamedCommands.registerCommand("SetWristB_1C_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false,2.93,6.84)) ));
      NamedCommands.registerCommand("SetWristB_1C_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(0,false, 5.48, 6.42)) ));
      NamedCommands.registerCommand("SetWristB_1C_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(0,false, 5.48, 6.42)) ));

      NamedCommands.registerCommand("SetWristB_2_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(0.0,false,2.64, 5.57)) ));
      NamedCommands.registerCommand("SetWristB_2_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false,3.83, 5.47)) ));
      NamedCommands.registerCommand("SetWristB_2_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(0.0,false,3.66,5.1 )) ));
      NamedCommands.registerCommand("SetWristB_2_4", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false,6.26, 1.38)) ));

      NamedCommands.registerCommand("SetWristB_3_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.2,false, 2.65, 4.24)) ));
      NamedCommands.registerCommand("SetWristB_3_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.2,false, 3.51, 2.99)) ));
      NamedCommands.registerCommand("SetWristB_3_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.2,false, 3.51, 2.99)) ));
     
      NamedCommands.registerCommand("SetWristB_4_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.3,false,2.74, 6.94) )));
      NamedCommands.registerCommand("SetWristB_4_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.3,false, 3.57,6.46))));
      NamedCommands.registerCommand("SetWristB_4_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false, 3.64,6.49)) ));
    
      NamedCommands.registerCommand("SetWristB_7_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.4,false, 4.18, 1.79)) ));
      NamedCommands.registerCommand("SetWristB_7_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false, 3.52, 2.83)) ));
      NamedCommands.registerCommand("SetWristB_7_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false, 3.52, 2.83)) ));
   
      NamedCommands.registerCommand("SetWristB_8_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.3,false,4.56,6.3)) ));//all -0.5 previously 
      NamedCommands.registerCommand("SetWristB_8_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false,4.96,6.36)) ));
      NamedCommands.registerCommand("SetWristB_8_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false,4.96 ,6.36)) ));
      NamedCommands.registerCommand("SetWristB_8_4", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false,4.96,6.36)) ));

      NamedCommands.registerCommand("SetWristB_8B_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.2,false,4.96 ,6.36)) ));
      NamedCommands.registerCommand("SetWristB_8B_4", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false,4.96 ,6.36)) ));

      NamedCommands.registerCommand("SetWristB_9_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(0, false,4,1.18)) ));
      NamedCommands.registerCommand("SetWristB_9_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5, false,4.76,1.62)) ));
      NamedCommands.registerCommand("SetWristB_9_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5, false,4.76,1.62)) ));
      NamedCommands.registerCommand("SetWristB_9_4", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5, false,1.27,4.51)) ));

      
      NamedCommands.registerCommand("SetWristB_10_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false, 4.21, 6.18)) ));
      NamedCommands.registerCommand("SetWristB_10_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false, 4.21, 6.18)) ));
      NamedCommands.registerCommand("SetWristB_10_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false,4.4 ,6.4)) ));
      NamedCommands.registerCommand("SetWristB_10_4", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false,3.91,2.64)) ));
      NamedCommands.registerCommand("SetWristB_10_5", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,false,4.09,2.62)) ));
      


      NamedCommands.registerCommand("SetWristR_1_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,true,13.57,6.95)) ));  // tuned 319
      NamedCommands.registerCommand("SetWristR_1_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.4,true,12.9,6.02)) ));  // tuned
      NamedCommands.registerCommand("SetWristR_1_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.2,true,12.9,6.02)) ));  // tuned

      NamedCommands.registerCommand("SetWristR_1B_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,true,13.57,6.95)) ));  // tuned 319
      NamedCommands.registerCommand("SetWristR_1B_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.4,true,12.9,6.02)) ));  // tuned
      NamedCommands.registerCommand("SetWristR_1B_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.2,true,12.9,6.02)) ));  // tuned

      NamedCommands.registerCommand("SetWristR_1C_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,true,13.57,6.95)) ));  
      NamedCommands.registerCommand("SetWristR_1C_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(0,true,11.02,6.42)) )); 
      NamedCommands.registerCommand("SetWristR_1C_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(0,true,11.02,6.42)) )); 

      NamedCommands.registerCommand("SetWristR_2_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(0.0,true,13.87, 5.57)) ));
      NamedCommands.registerCommand("SetWristR_2_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,true,12.67, 5.47)) ));
      NamedCommands.registerCommand("SetWristR_2_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(0.0,true,13.6,5.18 )) ));
      NamedCommands.registerCommand("SetWristR_2_4", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,true,10.24, 1.38)) ));
     
      NamedCommands.registerCommand("SetWristR_3_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.05,true,13.86,4.24)) ));  // tuned
      NamedCommands.registerCommand("SetWristR_3_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.2,true, 12.99, 2.99)) ));
      NamedCommands.registerCommand("SetWristR_3_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.2,true, 12.99, 2.99)) ));
     
      NamedCommands.registerCommand("SetWristR_4_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.1,true, 13.79, 6.86)) ));
      NamedCommands.registerCommand("SetWristR_4_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.4,true, 12.10,6.6))));
      NamedCommands.registerCommand("SetWristR_4_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.3,true, 12.10,6.6)) ));
      NamedCommands.registerCommand("SetWristR_4_4", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.3,true, 12.41,6.48)) ));

      NamedCommands.registerCommand("SetWristR_7_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.4,true,12.79,2.27)) ));
      NamedCommands.registerCommand("SetWristR_7_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,true,13.27,2.59)) ));
      NamedCommands.registerCommand("SetWristR_7_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5,true,13.01,2.56)) ));

      NamedCommands.registerCommand("SetWristR_8_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.3, true,12,6.32)) ));
      NamedCommands.registerCommand("SetWristR_8_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5, true,11.54,6.36)) ));
      NamedCommands.registerCommand("SetWristR_8_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5, true,11.54 ,6.36)) ));
      NamedCommands.registerCommand("SetWristR_8_4", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5, true,11.54,6.36)) ));
     
      NamedCommands.registerCommand("SetWristR_8B_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.2, true,11.54 ,6.36)) ));
      NamedCommands.registerCommand("SetWristR_8B_4", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5, true,11.54 ,6.36)) ));

      NamedCommands.registerCommand("SetWristR_9_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(0, true,12,1.18)) ));
      NamedCommands.registerCommand("SetWristR_9_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5, true,11.74,1.62)) ));
      NamedCommands.registerCommand("SetWristR_9_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5, true,11.74,1.62)) ));
      NamedCommands.registerCommand("SetWristR_9_4", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(-0.5, true,15.24,4.51)) ));

      NamedCommands.registerCommand("SetWristR_10_1", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(0.0,true, 12.29, 6.18)) ));
      NamedCommands.registerCommand("SetWristR_10_2", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(0.0,true, 12.02, 6.20)) ));
      NamedCommands.registerCommand("SetWristR_10_3", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(0.0,true, 12.10 ,6.4)) ));
      NamedCommands.registerCommand("SetWristR_10_4", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(0.0,true,12.59,2.64)) ));
      NamedCommands.registerCommand("SetWristR_10_5", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(0.0,true,12.41,2.62)) ));

      NamedCommands.registerCommand("SetWristR_11", new SequentialCommandGroup(new InstantCommand(() ->  wristSubsystem.moveWristAuto(0.0,true, 14.31, 3.25)) ));


      NamedCommands.registerCommand("StopVision", new SequentialCommandGroup(new InstantCommand(() -> wristSubsystem.stopMoveWristToTarget())));

    //  NamedCommands.registerCommand("Blu_10_Command", Blu_10_Command);
      NamedCommands.registerCommand("FireNote", new SequentialCommandGroup(new IntakeShootStateMachineOneShotCommand(intakeShootStateMachine, ISInput.JUST_SHOOT),
                                                                           new InstantCommand(() -> wristSubsystem.stopMoveWristToTarget())));
     // NamedCommands.registerCommand("JustShoot", new SequentialCommandGroup(new IntakeShootStateMachineOneShotCommand(intakeShootStateMachine, ISInput.JUST_SHOOT)));
      NamedCommands.registerCommand("IntakeNoJiggle", new SequentialCommandGroup(new IntakeShootStateMachineOneShotCommand(intakeShootStateMachine, ISInput.STOP_SPEAKER), 
                                                                                 new InstantCommand(() ->  wristSubsystem.moveWrist(0)),
                                                                                 new IntakeShootStateMachineOneShotCommand(intakeShootStateMachine, ISInput.INTAKE_NO_JIGGLE)));
    
   // buildAuto10();
   //  NamedCommands.registerCommand("Red_10_Command", Red_10_Command);
   //  NamedCommands.registerCommand("Blu_10_Command", Blu_10_Command);

    }
    
    climbStateMachine.setInitialState(CState.ROBOT_LATCHED_ON_CHAIN);

    intakeShootStateMachine.setInitialState(ISState.ALL_STOP);
    
    // Configure the button bindings
    configureButtonBindings();
    intakeShootStateMachine.setXboxController(xboxController);
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
/* 
     new JoystickButton(m_driverController, Button.kB.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    
    new JoystickButton(m_driverController, Button.kA.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    new JoystickButton(m_driverController, Button.kY.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.zeroPosition(), m_robotDrive));
*/
        
    ky.whileTrue(new DriveToLocationCommand(driveSubsystem, wristSubsystem,visionSubsystem, m_ledstring, xboxController, false, false));

    kPOVUp.whileTrue(new DriveToTrapCommand(driveSubsystem,visionSubsystem));

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
    
    kStart.onTrue(new InstantCommand(() -> {
      driveSubsystem.seedFieldRelative(new Pose2d(1.47,5.51, new Rotation2d (0)));
      Rotation2d operatorPerspective = Robot.isRedAlliance()? new Rotation2d(Math.toRadians(180)): new Rotation2d(Math.toRadians(0));
      Pose2d resetPosition = Robot.isRedAlliance()? new Pose2d(15.03,5.51,operatorPerspective): new Pose2d(1.47,5.51,operatorPerspective);
      driveSubsystem.seedFieldRelative(resetPosition);
      driveSubsystem.setOperatorPerspectiveForward(operatorPerspective);  // Just a Hack
    }));

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

    operatorkStart.whileTrue(
                        new ParallelCommandGroup(
                            new IntakeShootStateMachineCommand(intakeShootStateMachine, ISInput.START_EJECT), 
                            new InstantCommand(() -> climbStateMachine.setCurrentInput(CInput.START_EJECT))))
                  .onFalse(
                        new ParallelCommandGroup( 
                            new IntakeShootStateMachineOneShotCommand(intakeShootStateMachine, ISInput.STOP_EJECT),
                            new InstantCommand(() -> climbStateMachine.setCurrentInput(CInput.STOP_EJECT))));


    // Far Shot
  //  operatorky.onTrue(new InstantCommand(() -> wristSubsystem.moveWrist(12)))  // this is now over the stage
  //      .onFalse(new InstantCommand(() -> wristSubsystem.moveWrist(0)));

    operatorky.whileTrue(new ParallelCommandGroup(new IntakeShootStateMachineCommand(intakeShootStateMachine, ISInput.START_LOBSHOT),
                            new DriveToLocationCommand(driveSubsystem, wristSubsystem,visionSubsystem,m_ledstring, xboxController, true, false), new InstantCommand(() -> intakeShootStateMachine.setLobSpeed(50.0))))
              .onFalse(new IntakeShootStateMachineOneShotCommand(intakeShootStateMachine, ISInput.STOP_LOBSHOT));

    // Safe Shot
    operatorkb.onTrue(new InstantCommand(() -> wristSubsystem.moveWrist(22*0.6)))
        .onFalse(new InstantCommand(() -> wristSubsystem.moveWrist(0)));
    // Line Shot
    // operatorka.onTrue(new InstantCommand(() -> wristSubsystem.moveWrist(15*0.6)))
    //     .onFalse(new InstantCommand(() -> wristSubsystem.moveWrist(0)));
    operatorka.whileTrue(new ParallelCommandGroup(new IntakeShootStateMachineCommand(intakeShootStateMachine, ISInput.START_LOBSHOT),
                            new DriveToLocationCommand(driveSubsystem, wristSubsystem,visionSubsystem, m_ledstring, xboxController, true, true), new InstantCommand(() -> intakeShootStateMachine.setLobSpeed(45.0))))
              .onFalse(new IntakeShootStateMachineOneShotCommand(intakeShootStateMachine, ISInput.STOP_LOBSHOT));
    // operatorkRightTrigger.onTrue(new JiggleCommand(intakeShootSubsystem, shooterSubsystem));

    operatorBack.whileTrue(new InstantCommand(() -> visionSubsystem.setConfidence(true)));
    operatorBack.whileFalse(new InstantCommand(() -> visionSubsystem.setConfidence(false)));
        
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

  public Command getNamedAutonomousCommand(String autoName, boolean redAlliance) {
    String alliancePathName = autoName;
    if(!autoName.startsWith("Red_") && !autoName.startsWith("Blu_")){
        alliancePathName = (redAlliance ? "Red" : "Blu") + "_" + autoName;
    }
    // if the named auto (red or blue) exists, use it as-is and do NOT flip the field (red/blue)
    if(autoPaths.keySet().contains(alliancePathName)){
      flipRedBlue = false;
    }
    // if the named auto does not exist (so there isn't a red one), use the blue one and flip the field
    else if(redAlliance && alliancePathName.startsWith("Red_")) {
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
/* 
public  void buildAuto10() {

Blu_10_Command =
        Commands.sequence(
            driveSubsystem.getAutoPath("B_10_Pickup_1"),  // p1
            Commands.either(
                driveSubsystem.getAutoPath("B_10_Score1_Pickup2"), //p2, p3
                driveSubsystem.getAutoPath("B_10_Pickup_2"),  // p4
                intakeSubsystem::hasNote),
            Commands.either(
                driveSubsystem.getAutoPath("B_10_Score2_Pickup3"), //p5, p6
                driveSubsystem.getAutoPath("B_10_Pickup_3"), //p7
                intakeSubsystem::hasNote),
            Commands.either(
                driveSubsystem.getAutoPath("B_10_Score3_Pickup4"), //p8, p9
                driveSubsystem.getAutoPath("B_10_Pickup_4"), //p10
                intakeSubsystem::hasNote),
           Commands.either(
                driveSubsystem.getAutoPath("B_10_Score4_Pickup5"), //p11, p12
                driveSubsystem.getAutoPath("B_10_Pickup_5"), //p13
                intakeSubsystem::hasNote),
           Commands.either(
                driveSubsystem.getAutoPath("B_10_Score_5"), //14
                Commands.none(), 
                intakeSubsystem::hasNote));



Red_10_Command =
        Commands.sequence(
            driveSubsystem.getAutoPath("R_10_Pickup_1"),  // p1
            Commands.either(
                driveSubsystem.getAutoPath("R_10_Score1_Pickup2"), //p2, p3
                driveSubsystem.getAutoPath("R_10_Pickup_2"),  // p4
                intakeSubsystem::hasNote),
            Commands.either(
                driveSubsystem.getAutoPath("R_10_Score2_Pickup3"), //p5, p6
                driveSubsystem.getAutoPath("R_10_Pickup_3"), //p7
                intakeSubsystem::hasNote),
            Commands.either(
                driveSubsystem.getAutoPath("R_10_Score3_Pickup4"), //p8, p9
                driveSubsystem.getAutoPath("R_10_Pickup_4"), //p10
                intakeSubsystem::hasNote),
           Commands.either(
                driveSubsystem.getAutoPath("R_10_Score4_Pickup5"), //p11, p12
                driveSubsystem.getAutoPath("R_10_Pickup_5"), //p13
                intakeSubsystem::hasNote),
           Commands.either(
                driveSubsystem.getAutoPath("R_10_Score_5"), //14
                Commands.none(), 
                intakeSubsystem::hasNote));
                

}
*/
}
