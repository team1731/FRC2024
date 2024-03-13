/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.TunerConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command to fire into the speaker
 */
public class DriveToSpeakerCommand extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final CommandSwerveDrivetrain m_drivetrain;
	private WristSubsystem m_WristSubsystem;
	private CommandXboxController m_XboxController;
	private VisionSubsystem m_visionSubsystem;
		  private double MaxAngularRate = 1.5 * Math.PI;
		private  double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; 
		  private  SwerveRequest.FieldCentricFacingAngle driveAtSpeaker =  new SwerveRequest.FieldCentricFacingAngle().withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDeadband((MaxSpeed * 0.05));





	/**
	 * Creates a new Fire into the speaker
	 *
	 * @param CommandSwerveDrivetrain
	 */
	public DriveToSpeakerCommand(CommandSwerveDrivetrain drivetrain, WristSubsystem wristSubsystem, VisionSubsystem visionSubsystem, CommandXboxController xboxController) {
		m_drivetrain = drivetrain;
		m_WristSubsystem = wristSubsystem;
		m_XboxController = xboxController;
		m_visionSubsystem = visionSubsystem;


		// Use addRequirements() here to declare subsystem dependencies.
		if (drivetrain != null ) {
			addRequirements(drivetrain);
		}
	}

	// Called when the command is initially scheduled.
	// If it is used as Default command then it gets call all the time
	@Override
	public void initialize() {

       driveAtSpeaker.HeadingController.setPID(20,0,0);
	   driveAtSpeaker.HeadingController.enableContinuousInput(-Math.PI/2, Math.PI/2);
	  


    }
  

	


	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		m_WristSubsystem.setWristBasedOnDistance(m_visionSubsystem.getDistanceToSpeakerInMeters(m_visionSubsystem.adjustedRobotPose()), m_visionSubsystem.getXDistanceToSpeaker(), m_visionSubsystem.getYDistanceToSpeaker(), m_XboxController.getLeftX(), m_XboxController.getLeftY(), m_visionSubsystem.getAccelerationX(), m_visionSubsystem.getAccelerationY(), m_visionSubsystem.getHeadingAngleToSpeaker());
		m_drivetrain.setControl( 
			 driveAtSpeaker.withVelocityX(-(Math.abs(m_XboxController.getLeftY()) * m_XboxController.getLeftY()) * MaxSpeed)                                                                                                                     
                  .withVelocityY(-(Math.abs(m_XboxController.getLeftX()) * m_XboxController.getLeftX()) * MaxSpeed).withTargetDirection(m_visionSubsystem.getHeadingToSpeakerInRad())) 
                
          ;
		  SmartDashboard.putNumber("PID Setpoint", driveAtSpeaker.HeadingController.getSetpoint());
		  SmartDashboard.putNumber("PID Output", driveAtSpeaker.HeadingController.getLastAppliedOutput());
		  SmartDashboard.putNumber("PID Error", driveAtSpeaker.HeadingController.getPositionError());
		  SmartDashboard.putNumber("Current Heading", m_drivetrain.getState().Pose.getRotation().getRadians());


		  
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_WristSubsystem.moveWrist(0);

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}