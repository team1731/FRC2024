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
import frc.robot.subsystems.VisionSubsystem;

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
	private final VisionSubsystem m_visionSubsystem;
	private boolean runningPath = false;
	private Pose2d waypointPose;
	private Pose2d finalPose;
	private Command pathFollowingCommand;

	private PosePair[] STAGE_POSES = {
        //                                      WAYPOINT POSE                FINAL POSE
		/* BLUE LEFT   */ new PosePair(new Pose( 4.20, 5.28,  -60), new Pose( 4.46, 4.82,  -60)),
		/* BLUE RIGHT  */ new PosePair(new Pose( 4.24, 2.96,   60), new Pose( 4.44, 3.38,   60)),
		/* BLUE CENTER */ new PosePair(new Pose( 6.28, 4.13,  180), new Pose( 5.72, 4.13,  180)),
		/* RED LEFT    */ new PosePair(new Pose(12.37, 2.94,  120), new Pose(12.13, 3.39,  120)),
		/* RED RIGHT   */ new PosePair(new Pose(12.37, 5.31, -120), new Pose(12.13, 4.84, -120)),
		/* RED CENTER  */ new PosePair(new Pose( 10.31, 4.10,   0), new Pose(10.84, 4.10,    0))
	};


	/**
	 * Creates a new Fire into the speaker
	 *
	 * @param CommandSwerveDrivetrain
	 */
	public DriveToTrapCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem) {
		m_drivetrain = drivetrain;
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
	  //
	  // It may be desirable to use a "canned" path approach (left here for possible future consideration):
	  // PathPlannerPath path = PathPlannerPath.fromPathFile("paths/Chain.path");
	  //
	  runningPath = false;
      m_visionSubsystem.drivingToTrap();
	  Pose2d currentPose = m_drivetrain.getState().Pose;
	  Pose2d[] closestPoses = getClosestChainPoses(currentPose);
	  if(closestPoses == null){
		return;
	  }

	  waypointPose = closestPoses[0];
      finalPose = closestPoses[1];
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(),currentPose.getRotation());
      
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, waypointPose, finalPose);
      PathPlannerPath path = new PathPlannerPath(
        bezierPoints, 
        new PathConstraints(
          2.0, 2.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),  
        new GoalEndState(0.0, finalPose.getRotation())
      );

      // Prevent this path from being flipped on the red alliance, since the given positions are already correct
      path.preventFlipping = true;
	  
	  pathFollowingCommand = AutoBuilder.followPath(path);
	  System.out.println("Initialization complete");

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
		if(closestPoses == null){
			System.err.println("\n\nERROR: cound not find a pose to nearest chain!\n\n");
		}
		else{
			Pose2d closest = closestPoses[0];
			System.out.println("\nFOUND pose to nearest chain at " + closest.getX() + ", " + closest.getY() + ", " + closest.getRotation().getDegrees());
		}
		return closestPoses;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		if (m_visionSubsystem.haveGoodVisionLock() && !runningPath) {
			runningPath = true;
			System.out.println("Goin for a drive");
			pathFollowingCommand.schedule();

			
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_visionSubsystem.stopDrivingToTrap();
		runningPath = false;
		
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}