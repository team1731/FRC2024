/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Robot;
import frc.robot.TunerConstants;
import frc.robot.Constants.OpConstants.LedOption;
import frc.robot.subsystems.LEDStringSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command to fire into the speaker
 */
public class DriveToLocationCommand extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final CommandSwerveDrivetrain m_drivetrain;
	private WristSubsystem m_WristSubsystem;
	private CommandXboxController m_XboxController;
	private VisionSubsystem m_visionSubsystem;
	private LEDStringSubsystem m_ledSubsystem;
	private double MaxAngularRate = 1.5 * Math.PI;
	private  double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; 
	private  SwerveRequest.FieldCentricFacingAngle driveAtLocation =  new SwerveRequest.FieldCentricFacingAngle().withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband((MaxSpeed * 0.05));
	private boolean m_lobShot;
	private boolean m_lobShotAtOppositeStage;
	private double targetAngle;

	/**
	 * Creates a new Fire into the speaker
	 * @param b 
	 *
	 * @param CommandSwerveDrivetrain
	 */
	public DriveToLocationCommand(CommandSwerveDrivetrain drivetrain, WristSubsystem wristSubsystem, VisionSubsystem visionSubsystem, LEDStringSubsystem ledSubsystem, CommandXboxController xboxController, boolean lobShot, boolean lobShotFromStage) {
		m_drivetrain = drivetrain;
		m_WristSubsystem = wristSubsystem;
		m_XboxController = xboxController;
		m_visionSubsystem = visionSubsystem;
		m_lobShot = lobShot;
		m_lobShotAtOppositeStage = lobShotFromStage;
		m_ledSubsystem = ledSubsystem;

		// Use addRequirements() here to declare subsystem dependencies.
		if (drivetrain != null ) {
			addRequirements(drivetrain);
		}
	}

	// Called when the command is initially scheduled.
	// If it is used as Default command then it gets call all the time
	@Override
	public void initialize() {
       driveAtLocation.HeadingController.setPID(10,0,0);
	   driveAtLocation.HeadingController.enableContinuousInput(-Math.PI/2, Math.PI/2);
	   if (m_lobShot) {
		m_ledSubsystem.setColor(LedOption.YELLOW);
	   }
    }
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (m_lobShot) {
			m_WristSubsystem.moveWrist(7);
			if (m_lobShotAtOppositeStage){ 
				targetAngle = Robot.isRedAlliance() ? Math.toRadians(20) : Math.toRadians(160);
			}else{
				targetAngle = Robot.isRedAlliance() ? Math.toRadians(40) : Math.toRadians(140);
			}
			Rotation2d lobRotation = new Rotation2d(targetAngle);
			m_drivetrain.setControl( 
				driveAtLocation.withVelocityX(-(Math.abs(m_XboxController.getLeftY()) * m_XboxController.getLeftY()) * MaxSpeed)                                                                                                                     
					.withVelocityY(-(Math.abs(m_XboxController.getLeftX()) * m_XboxController.getLeftX()) * MaxSpeed).withTargetDirection(lobRotation));
		}
		 else {
			m_WristSubsystem.setWristBasedOnDistance(m_visionSubsystem.getDistanceToSpeakerInMeters());
			m_drivetrain.setControl( 
				driveAtLocation.withVelocityX(-(Math.abs(m_XboxController.getLeftY()) * m_XboxController.getLeftY()) * MaxSpeed)                                                                                                                     
					.withVelocityY(-(Math.abs(m_XboxController.getLeftX()) * m_XboxController.getLeftX()) * MaxSpeed).withTargetDirection(m_visionSubsystem.getHeadingToSpeakerInRad()));
					
		}
		SmartDashboard.putNumber("PID Setpoint", driveAtLocation.HeadingController.getSetpoint());
		SmartDashboard.putNumber("PID Output", driveAtLocation.HeadingController.getLastAppliedOutput());
		SmartDashboard.putNumber("PID Error", driveAtLocation.HeadingController.getPositionError());
		SmartDashboard.putNumber("Current Heading", m_drivetrain.getState().Pose.getRotation().getRadians());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_WristSubsystem.moveWrist(0);
		m_ledSubsystem.setColor(LedOption.BLACK);

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}