package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ToggleableSubsystem;

class FlipRedBlueSupplier implements BooleanSupplier {
    @Override
    public boolean getAsBoolean() {
        return RobotContainer.isFlipRedBlue();
    }
}

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements ToggleableSubsystem {
    // private static final double kSimLoopPeriod = 0.005; // 5 ms
    // private Notifier m_simNotifier = null;
    // private double m_lastSimTime;
    private boolean enabled;
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    
    private void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public CommandSwerveDrivetrain(boolean enabled, SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        setEnabled(enabled);
        if(!enabled) return;
        configurePathPlanner(true);
    }
    
    public CommandSwerveDrivetrain(boolean enabled, SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        setEnabled(enabled);
        if(!enabled) return;
        configurePathPlanner(true);
    }

    public void configurePathPlanner(boolean redBlueFlipping) {
        if(!enabled) return;
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        System.out.println("Configuring AutoBuilder!");

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            new FlipRedBlueSupplier(), // ()->false, // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        if(!enabled) return new Command(){};
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        if(!enabled) return new Command(){};
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        if(!enabled || Robot.isSimulation()) return new ChassisSpeeds();
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

/**
Log the torque current and velocity
 */
    public void logCurrentAndVelocity() {
       

        //Iterate through each module.
        for (int i = 0; i < 4; ++i) {
            //Get the Configurator for the current drive motor.
            SmartDashboard.putNumber("Module " + i + "Torque Current" ,Modules[i].getDriveMotor().getTorqueCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Module " + i + "Velocity" ,Modules[i].getDriveMotor().getVelocity().getValueAsDouble());
        }
    }

        /** Log various drivetrain values to the dashboard. */
    public void log() {
        // String table = "Drive/";
        // Pose2d pose = this.getState().Pose;
        // SmartDashboard.putNumber(table + "X", pose.getX());
        // SmartDashboard.putNumber(table + "Y", pose.getY());
        // SmartDashboard.putNumber(table + "Heading", pose.getRotation().getDegrees());
        // ChassisSpeeds chassisSpeeds = getCurrentRobotChassisSpeeds();
        // SmartDashboard.putNumber(table + "VX", chassisSpeeds.vxMetersPerSecond);
        // SmartDashboard.putNumber(table + "VY", chassisSpeeds.vyMetersPerSecond);
        // SmartDashboard.putNumber(
        //         table + "Omega Degrees", Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond));
        // SmartDashboard.putNumber(table + "Target VX", targetChassisSpeeds.vxMetersPerSecond);
        // SmartDashboard.putNumber(table + "Target VY", targetChassisSpeeds.vyMetersPerSecond);
        // SmartDashboard.putNumber(
        //         table + "Target Omega Degrees", Math.toDegrees(targetChassisSpeeds.omegaRadiansPerSecond));

        // for (SwerveModule module : swerveMods) {
        //     module.log();
        // }
    }

}
