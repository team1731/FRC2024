package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import frc.robot.commands.AutoBalanceSwerve;
import frc.robot.commands.AutoCheckRemainingTime;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.Swerve;

// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PIDConstants;

import java.util.HashMap;
import java.util.List;

// import com.pathplanner.lib.PathConstraints;

public class PathPlannerCommandGroup extends SequentialCommandGroup {
    private String pathName;

    public String toString(){
        return getClass().getSimpleName() + ": =====>>> " + pathName + " <<<=====";
    }

    public PathPlannerCommandGroup(String pathPlannerFile,  Swerve s_Swerve, PoseEstimatorSubsystem s_PoseEstimatorSubsystem, double maxVelocity, double maxAcceleration) {
        pathName = pathPlannerFile;
        // This will load the file pathPlannerFile and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the group
      //  PathConstraints pathConstraints = new PathConstraints(4, 2.0); //Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        // List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathPlannerFile, new PathConstraints(maxVelocity, maxAcceleration)); // "A1"
    
        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        //eventMap.put("ScoreCone", new PrintCommand("Passed marker 1"));
        eventMap.put("AutoBalanceSwerve", new AutoBalanceSwerve(s_Swerve));
        eventMap.put("AutoScoreHighWait", new WaitCommand(2.2));
        eventMap.put("SetVisionPipelineCube", new InstantCommand(() -> s_PoseEstimatorSubsystem.setVisionPipeline(GamePiece.CUBE)));
        eventMap.put("SetVisionPipelineCone", new InstantCommand(() -> s_PoseEstimatorSubsystem.setVisionPipeline(GamePiece.CONE)));
        eventMap.put("EnableVisionCorrection", new InstantCommand(() -> s_PoseEstimatorSubsystem.enableVisionCorrection()));
        eventMap.put("DisableVisionCorrection", new InstantCommand(() -> s_PoseEstimatorSubsystem.disableVisionCorrection()));
        eventMap.put("EnableAprilTags", new InstantCommand(() -> s_PoseEstimatorSubsystem.enableAprilTags(true)));
        eventMap.put("DisableAprilTags", new InstantCommand(() -> s_PoseEstimatorSubsystem.enableAprilTags(false)));
        //eventMap.put("intakeDown", new IntakeDown());
    
        // // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        AutoBuilder autoBuilder = new AutoBuilder();
        // autoBuilder.configureRamsete(
        //     s_PoseEstimatorSubsystem::getAutoPose, // Pose2d supplier
        //     s_PoseEstimatorSubsystem::setCurrentPose, // Pose2d consumer, used to reset odometry at the beginning of auto
        //     Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
        //     new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        //     new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0), //Constants.Swerve.angleKD), // PID constants to correct for rotation error (used to create the rotation controller)
        //     s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
        //     eventMap,
        //     false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        //     s_Swerve,
        //     s_PoseEstimatorSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
        // );
    
        Command fullAuto = null; //autoBuilder.fullAuto(pathGroup);
        System.out.println("PathPlannerCommandGroup: pathPlannerFile '" + pathPlannerFile + "' has been planned.");
        // addCommands(fullAuto);
    }
}
