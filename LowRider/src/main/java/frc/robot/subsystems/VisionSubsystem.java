/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants.OpConstants.LedOption;
import frc.robot.util.log.Logger;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.hardware.Pigeon2;

public class VisionSubsystem extends SubsystemBase implements ToggleableSubsystem {
    private PhotonCamera cameraFront;
    private PhotonCamera cameraBack;

    private PhotonPoseEstimator photonEstimatorFront;
    private PhotonPoseEstimator photonEstimatorBack;
    private final Field2d field2d = new Field2d();
    private Pose2d redGoal = new Pose2d(new Translation2d(16.579342,5.547868), new Rotation2d());
    private Pose2d blueGoal = new Pose2d(new Translation2d(0.0381,5.547868), new Rotation2d());
    private boolean useVision = false;
    private LEDStringSubsystem ledSubsystem;   

    private CommandSwerveDrivetrain m_driveSubsystem;
    private double lastEstTimestampFront;
    private double lastEstTimestampBack;

    // logging
    Logger poseLogger;
    double lastLogTime = 0;
    double logInterval = 1.0; // in seconds

    Pigeon2 mypigeon;
    private boolean enabled;

    private double shootOnMoveFudgeFactor = 1.2;

    @Override
    public boolean isEnabled() {
        return enabled;
    }

    private boolean initialized;

    public boolean isInitialized() {
        return initialized;
    }

    public VisionSubsystem(boolean enabled, CommandSwerveDrivetrain driveSubsystem, LEDStringSubsystem ledsubsystem) {
        this.enabled = enabled;
        this.m_driveSubsystem = driveSubsystem;
        this.ledSubsystem = ledsubsystem;
        mypigeon = m_driveSubsystem.getPigeon2();
        cameraFront = null;
        cameraBack = null;
        photonEstimatorFront = null;
        photonEstimatorBack = null;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // get the subtable called "photonvision"
        NetworkTable photonVisionTable = inst.getTable("photonvision/" + kCameraNameFront);
        if (photonVisionTable.containsKey("hasTarget")) {
            cameraFront = new PhotonCamera(kCameraNameFront);
            photonEstimatorFront = new PhotonPoseEstimator(
                kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraFront, kRobotToCamFront);
            photonEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            initialized = true;
            System.out.println("VisionSubsystem: Adding camera " + kCameraNameFront + "!!!!!!! ");
        } 
        
        photonVisionTable = inst.getTable("photonvision/" + kCameraNameBack);
        if (photonVisionTable.containsKey("hasTarget")) {
            cameraBack = new PhotonCamera(kCameraNameBack);
            photonEstimatorBack = new PhotonPoseEstimator(
                kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraBack, kRobotToCamBack);
            photonEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            initialized = true;
            System.out.println("VisionSubsystem: Adding camera " + kCameraNameBack + "!!!!!!! ");
        }
        
        if (!initialized) {
            System.out.println("VisionSubsystem: Init FAILED: " + " Keys: " + photonVisionTable.getKeys().toString());
        }

        // write initial values to dashboard
        if(enabled){
            ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
            String formattedPose = this.getFormattedPose();
            if (formattedPose != null) {
                tab.addString("Pose (X, Y)", this::getFormattedPose).withPosition(0, 4);
            }
            Pose2d currentPose = this.getCurrentPose();
            if (currentPose != null) {
                tab.addNumber("Pose Degrees", () -> currentPose.getRotation().getDegrees()).withPosition(1, 4);
            }
            tab.add(field2d);
        }

        // setup logger
        // poseLogger = LogWriter.getLogger(Log.POSE_ESTIMATIONS,
        // PoseEstimations.class);
        // setCurrentPose(new Pose2d(1.88,5.01,new Rotation2d()));
    }

    private String getFormattedPose() {
        if (enabled) {
            var pose = getCurrentPose();
            return String.format("(%.2f, %.2f)", Units.metersToInches(pose.getX()), Units.metersToInches(pose.getY()));
        } else {
            return null;
        }
    }

    public Pose2d getCurrentPose() {
        if (enabled) {
            return m_driveSubsystem.getState().Pose;
        } else {
            return null;
        }
    }

    @Override
    public void periodic() {

        getDistanceToSpeakerInMeters();   // probably want to comment this out after testing
        if (enabled && initialized) {
        
            if (photonEstimatorFront != null) {
                // Correct pose estimate with vision measurements
                var visionEstFront = getEstimatedGlobalPoseFront();
                visionEstFront.ifPresent(
                        est -> {
                            var estPose = est.estimatedPose.toPose2d();
                            // Change our trust in the measurement based on the tags we can see
                            var estStdDevs = getEstimationStdDevs(cameraFront, estPose, photonEstimatorFront);
                            field2d.getObject("MyRobot" + cameraFront.getName()).setPose(estPose);
                            SmartDashboard.putString("Vision pose", String.format("(%.2f, %.2f) %.2f",
                                estPose.getTranslation().getX(),
                                estPose.getTranslation().getY(),
                                estPose.getRotation().getDegrees()));


                            //System.out.println("VisionFront(" + est.timestampSeconds + "): " + est.estimatedPose.toPose2d().getX() + "-" + estStdDevs.getData().toString() );
                            if (useVision) {
                                m_driveSubsystem.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                                lastEstTimestampFront = Timer.getFPGATimestamp();
                            }
                        });
            }

            if (photonEstimatorBack != null) {
                // Correct pose estimate with vision measurements
                var visionEstBack = getEstimatedGlobalPoseBack();
                visionEstBack.ifPresent(
                        est -> {
                            var estPose = est.estimatedPose.toPose2d();
                            // Change our trust in the measurement based on the tags we can see
                            var estStdDevs = getEstimationStdDevs(cameraBack, estPose, photonEstimatorBack);

                            //System.out.println("VisionBack(" + est.timestampSeconds + "): " + est.estimatedPose.toPose2d().getX() + "-" + estStdDevs.getData().toString() );
                            field2d.getObject("MyRobot" + cameraBack.getName()).setPose(estPose);
                            SmartDashboard.putString("Vision pose", String.format("(%.2f, %.2f) %.2f",
                                estPose.getTranslation().getX(),
                                estPose.getTranslation().getY(),
                                estPose.getRotation().getDegrees()));
                            if (useVision) {
                                m_driveSubsystem.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                                lastEstTimestampBack = Timer.getFPGATimestamp();
                            }
                        });
            }

            double curTime = Timer.getFPGATimestamp();
            // if both cameras are stale set warning
            if (((curTime - lastEstTimestampFront) > kTargetConfidenceDelta) && 
                ((curTime - lastEstTimestampBack) > kTargetConfidenceDelta)) {
                // System.out.println("false: " + targetConf);
                ledSubsystem.setWarning(true);
                SmartDashboard.putBoolean("Target Conf", false);
            } else {
                // System.out.println("true: " + targetConf);
                ledSubsystem.setWarning(false);
                SmartDashboard.putBoolean("Target Conf", true);
            }

            // // Apply a random offset to pose estimator to test vision correction
            // if (controller.getBButtonPressed()) {
            // var trf =
            // new Transform2d(
            // new Translation2d(rand.nextDouble() * 4 - 2, rand.nextDouble() * 4 - 2),
            // new Rotation2d(rand.nextDouble() * 2 * Math.PI));
            // m_driveSubsystem.resetPose(m_driveSubsystem.getPose().plus(trf), false);
            // }

            // Log values to the dashboard
            // m_driveSubsystem.log();
        }

        field2d.setRobotPose(getCurrentPose());
        // Pose2d posenow = getCurrentPose();
        // SmartDashboard.putNumber("CurrentPoseX", posenow.getX());
        // SmartDashboard.putNumber("CurrentPoseY", posenow.getY());

        // log pose estimations
        // Pose2d currentPose = getCurrentPose();
        // currentPose = getAutoPose(); // this is a hack - delete this line
        // poseLogger.add(new PoseEstimations(currentPose.getX(), currentPose.getY(),
        // currentPose.getRotation().getDegrees()));
        // if (Timer.getFPGATimestamp() - lastLogTime > logInterval) {
        //     // poseLogger.flush();
        //     lastLogTime = Timer.getFPGATimestamp();
        // }
    }

    // public Pose2d getAutoPose() {
    //     if (enabled) {
    //         if (useVisionCorrection) {
    //             return new Pose2d(); // getAdjustedPose();
    //         }
    //         return new Pose2d(); // poseEstimator.getEstimatedPosition();
    //     } else {
    //         return null;
    //     }
    // }

    // uses Pythagorean theorem to find the difference of two x and y coordinates
    // private double getDistanceDifference(double x1, double y1, double x2, double y2) {

    //     // Pythagorean theorem
    //     //--
    //     //  d=√((x_2-x_1)²+(y_2-y_1)²)
    //     //--

    //     return Math.sqrt(
    //         Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2)
    //     );
    // }

    public PhotonPipelineResult getLatestResult(PhotonCamera camera) {

        PhotonPipelineResult cameraResult = camera.getLatestResult();
        return cameraResult;
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate
     *         timestamp, and targets
     *         used for estimation.
     */
    private Optional<EstimatedRobotPose> getEstimatedGlobalPoseFront() {
        var visionEst = photonEstimatorFront.update();
        // double latestTimestamp = cameraFront.getLatestResult().getTimestampSeconds();
        // boolean newResult = Math.abs(latestTimestamp - lastEstTimestampFront) > 1e-5;
        // if (newResult)
        //     lastEstTimestampFront = latestTimestamp;
        return visionEst;
    }

    private Optional<EstimatedRobotPose> getEstimatedGlobalPoseBack() {
        var visionEst = photonEstimatorBack.update();
        // double latestTimestamp = cameraBack.getLatestResult().getTimestampSeconds();
        // boolean newResult = Math.abs(latestTimestamp - lastEstTimestampBack) > 1e-5;
        // if (newResult)
        //     lastEstTimestampBack = latestTimestamp;
        return visionEst;
    }

    /**
     * The standard deviations of the estimated pose from
     * {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(PhotonCamera camera, Pose2d estimatedPose,
            PhotonPoseEstimator photonEstimator) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = camera.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        
        if (numTags == 0) {
            ledSubsystem.setColor(LedOption.WHITE);
            return estStdDevs;
        }

        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            estStdDevs = kMultiTagStdDevs;
        }
        
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                
        return estStdDevs;
    }

    public Rotation2d getHeadingToSpeakerInRad() {
        Pose2d target = isRedAlliance()? redGoal: blueGoal;
        Pose2d robot = getAdjustedRobotPose();
        double headingToTarget = Math.atan((target.getY() - robot.getY())/(robot.getX() - target.getX()));
        SmartDashboard.putNumber("HeadingToTarget", headingToTarget);
      //  System.out.println("getting heading");
        return new Rotation2d(-headingToTarget);
    }

    public double getDistanceToSpeakerInMeters() {
        Pose2d target = isRedAlliance()? redGoal: blueGoal;
        Pose2d robot = getAdjustedRobotPose();
        double distance = PhotonUtils.getDistanceToPose(target, robot);
      //  SmartDashboard.putNumber("DistanceToTarget", distance);
        return distance;
    }

    public double getStaticDistanceToSpeakerInMeters() {
        Pose2d target = isRedAlliance()? redGoal: blueGoal;
        Pose2d robot = m_driveSubsystem.getState().Pose;
        double distance = PhotonUtils.getDistanceToPose(target, robot);
        SmartDashboard.putNumber("DistanceToTarget", distance);
        return distance;
    }

    public double getAccelerationX() {
        return (mypigeon.getAccelerationX().getValueAsDouble())*9.81;
    }
    public double getAccelerationY() {
        return (mypigeon.getAccelerationY().getValueAsDouble())*9.81;
    }

     public Pose2d getAdjustedRobotPose() {
        Pose2d robot = new Pose2d(m_driveSubsystem.getState().Pose.getX(), m_driveSubsystem.getState().Pose.getY(), m_driveSubsystem.getState().Pose.getRotation());

        
        double shotTime = (getStaticDistanceToSpeakerInMeters() / (3.81 * 2 * Math.PI)) * shootOnMoveFudgeFactor; //speed of shot in m/s

        SmartDashboard.putNumber("ShootOnMove Fudge Factor: ", shootOnMoveFudgeFactor);
        
        double robotXSpeed = m_driveSubsystem.getXVelocity();
        double robotYSpeed = m_driveSubsystem.getYVelocity();
        double robotXAcceleration = -getAccelerationY();
        double robotYAcceleration = getAccelerationX();

        Transform2d adjustment = new Transform2d(robotXSpeed*shotTime + 0.5*robotXAcceleration*shotTime*shotTime, robotYSpeed*shotTime +  0.5*robotYAcceleration*shotTime*shotTime, new Rotation2d());
       
        SmartDashboard.putNumber("x adjustment", robotXSpeed*shotTime + 0.5*robotXAcceleration*shotTime*shotTime);
        SmartDashboard.putNumber("y adjustment", robotYSpeed*shotTime + 0.5*robotYAcceleration*shotTime*shotTime);

        if (isRedAlliance()) {
            robot.rotateBy(new Rotation2d(Math.toRadians(180)));
        }

        SmartDashboard.putNumber("RobotXSpeed", robotXSpeed);
        SmartDashboard.putNumber("RobotYSpeed", robotYSpeed);
        SmartDashboard.putNumber("RobotXAcceleration", robotXAcceleration);
        SmartDashboard.putNumber("RobotYAcceleration", robotYAcceleration);


        robot.rotateBy(m_driveSubsystem.getState().Pose.getRotation());

        Pose2d adjustedRobotPose = robot.plus(adjustment);

        //robot.transformBy(adjustment);

        field2d.getObject("MyRobotAdjusted").setPose(adjustedRobotPose);

        return adjustedRobotPose;

    }

    public void shootOnMoveFudgeDown() {
        shootOnMoveFudgeFactor = shootOnMoveFudgeFactor - .1;
        System.out.println("NEW ShootOnMove FUDGE FACTOR: " + shootOnMoveFudgeFactor);
    }

    public void shootOnMoveFudgeUp() {
        shootOnMoveFudgeFactor = shootOnMoveFudgeFactor + .1;
        System.out.println("NEW ShootOnMove FUDGE FACTOR: " + shootOnMoveFudgeFactor);
    }

    private boolean isRedAlliance(){
	    Optional<Alliance> alliance = DriverStation.getAlliance();
	    if(alliance != null){
		    return alliance.get() == DriverStation.Alliance.Red;
	    }
	    return false;
    }

    public void useVision(boolean useCameraVision) {
        useVision = useCameraVision;
    }
}
