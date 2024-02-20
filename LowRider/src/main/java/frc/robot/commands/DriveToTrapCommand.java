/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import java.util.List;

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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command to fire into the speaker
 */
public class DriveToTrapCommand extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final CommandSwerveDrivetrain m_drivetrain;
	private Pose2d endPos;
	private Command pathFollowingCommand;




	/**
	 * Creates a new Fire into the speaker
	 *
	 * @param CommandSwerveDrivetrain
	 */
	public DriveToTrapCommand(CommandSwerveDrivetrain drivetrain) {
		m_drivetrain = drivetrain;


		// Use addRequirements() here to declare subsystem dependencies.
		if (drivetrain != null ) {
			//addRequirements(drivetrain);
		}
	}

	// Called when the command is initially scheduled.
	// If it is used as Default command then it gets call all the time
	@Override
	public void initialize() {

      Pose2d currentPose = m_drivetrain.getState().Pose;
	  endPos = getClosestChainPose(currentPose);

	  // find the 
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
        bezierPoints, 
        new PathConstraints(
          4.0, 4.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),  
        new GoalEndState(0.0, currentPose.getRotation())
      );

      // Prevent this path from being flipped on the red alliance, since the given positions are already correct
      path.preventFlipping = true;
	  
	  pathFollowingCommand = AutoBuilder.followPath(path);
      pathFollowingCommand.schedule();

    }
  

	

	private Pose2d getClosestChainPose(Pose2d currentPose) {
         // this is just returning a pose 2 meters to the left of the current pose - needs to be 
		 return new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

	
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}