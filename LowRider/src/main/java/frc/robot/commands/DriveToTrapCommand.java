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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;

class Pose {
	double x, y, degrees;
	public Pose(double x, double y, double degrees){
		this.x = x; this.y = y; this.degrees = degrees;
	}
	public Pose2d toPose2d(){
		return new Pose2d(x, y, new Rotation2d(Units.degreesToRadians(degrees)));
	}
}

class PosePair {
	Pose start, end;
	public PosePair(Pose start, Pose end){
		this.start = start; this.end = end;
	}
	public Pose2d[] toPose2dArray(){
		return new Pose2d[]{start.toPose2d(), end.toPose2d()};
	}
}

public class DriveToTrapCommand extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final CommandSwerveDrivetrain m_drivetrain;
	private Pose2d endPose;
	private Command pathFollowingCommand;

	private PosePair[] STAGE_POSES = {
        //                                      STARTING POSE                ENDING POSE
		/* BLUE LEFT   */ new PosePair(new Pose( 3.84, 5.81,  -60), new Pose( 4.43, 4.85,  -60)),
		/* BLUE RIGHT  */ new PosePair(new Pose( 4.12, 2.79,   60), new Pose( 4.40, 3.30,   60)),
		/* BLUE CENTER */ new PosePair(new Pose( 6.48, 4.17,  180), new Pose( 5.38, 4.17,  180)),
		/* RED LEFT    */ new PosePair(new Pose(12.65, 2.46,  120), new Pose(12.17, 3.33,  120)),
		/* RED RIGHT   */ new PosePair(new Pose(12.55, 5.75, -120), new Pose(12.08, 4.88, -120)),
		/* RED CENTER  */ new PosePair(new Pose( 9.59, 4.08,    0), new Pose(10.78, 4.08,    0))
	};


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
	  //
	  // It may be desirable to use a "canned" path approach (left here for possible future consideration):
	  // PathPlannerPath path = PathPlannerPath.fromPathFile("paths/Chain.path");
	  //

	  Pose2d currentPose = m_drivetrain.getState().Pose;
	  Pose2d[] closestPoses = getClosestChainPoses(currentPose);
	  endPose = closestPoses[0];
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPose);
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
  

	
	
	private Pose2d[] getClosestChainPoses(Pose2d currentPose) {
		Pose2d[] closestPoses = null;
		double shortestDistance = 100;
		for (PosePair posePair : STAGE_POSES) {
			Pose2d[] fieldPoses = posePair.toPose2dArray();
			double distanceToCurrent = currentPose.getTranslation().getDistance(fieldPoses[0].getTranslation());
			if(distanceToCurrent < shortestDistance){
				shortestDistance = distanceToCurrent;
				closestPoses = fieldPoses;
			}
		}
		return closestPoses;
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