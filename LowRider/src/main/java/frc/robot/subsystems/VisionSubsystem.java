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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Robot;
import frc.robot.util.log.Logger;

import java.util.EnumSet;
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
    private Pose2d blueGoal = new Pose2d(new Translation2d(-0.0381,5.547868), new Rotation2d());
    private boolean useVision = false;  
    private boolean useVSLAM = true;
    
    private Pose2d visionPose;

    private CommandSwerveDrivetrain m_driveSubsystem;
    private double lastEstTimestampFront;
    private double lastEstTimestampBack;
    private int visionInitCount;
    private boolean runningTrapPath;
    private boolean isZoomCameraReadingValid = false;

    /* VSLAM Updates */

    int connListenerHandle;
    int positionListenerHandle;
    int topicListenerHandle;

    // Configure Network Tables topics (oculus/...) to communicate with the Quest
    // HMD


    private IntegerSubscriber questMiso;
    private IntegerPublisher questMosi;

    // Subscribe to the Network Tables oculus data topics
    private IntegerSubscriber questFrameCount;
    private DoubleSubscriber questTimestamp;
    private FloatArraySubscriber questPosition;
    private FloatArraySubscriber questQuaternion;
    private FloatArraySubscriber questEulerAngles;
    private DoubleSubscriber questBattery;

    private float yaw_offset = 0.0f;

    // logging
    Logger poseLogger;
    double lastLogTime = 0;
    double logInterval = 1.0; // in seconds

    Pigeon2 mypigeon;
    private boolean enabled;
    private boolean confidence;
    private Pose2d startingOffset = new Pose2d();

    private double shootOnMoveFudgeFactor = 1.2;

    @Override
    public boolean isEnabled() {
        return enabled;
    }

    private boolean initialized;
    private boolean operatorOverrideConfidence;

    public boolean isConfident() {
        return confidence;
    }

    public boolean isInitialized() {
        return initialized;
    }

    public void visionInitialization(){
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
    }

    public VisionSubsystem(boolean enabled, CommandSwerveDrivetrain driveSubsystem) {
       
      

        this.enabled = enabled;
        this.m_driveSubsystem = driveSubsystem;
        mypigeon = m_driveSubsystem.getPigeon2();
        visionInitCount = 0;
        visionInitialization();

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
    }

    private String getFormattedPose() {
        if (enabled) {
            var pose = getCurrentPose();
            if(pose == null){
                return null;
            }
            else{
                return String.format("(%.2f, %.2f)", Units.metersToInches(pose.getX()), Units.metersToInches(pose.getY()));
            }
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

        if (!initialized) {
            //System.out.println("Checking vision, currently not initialized");
            if (visionInitCount++ >= 100) { // 20ms @ 50
                visionInitialization();
                visionInitCount = 0;
            }
        }

        getDistanceToSpeakerInMeters();   // probably want to comment this out after testing

        if (enabled && initialized) {

            if (photonEstimatorFront != null) {
                // Correct pose estimate with vision measurements
                try {
                    var visionEstFront = getEstimatedGlobalPoseFront();
                    isZoomCameraReadingValid  = visionEstFront.isPresent();
                    visionEstFront.ifPresent(
                            est -> {
                                var estPose = est.estimatedPose.toPose2d();
                                // Change our trust in the measurement based on the tags we can see
                                var estStdDevs = getEstimationStdDevs(cameraFront, estPose, photonEstimatorFront);
                                field2d.getObject("MyRobot" + cameraFront.getName()).setPose(estPose);
                                // SmartDashboard.put("vision standard deviation", estStdDevs));
                                SmartDashboard.putString("Vision pose", String.format("(%.2f, %.2f) %.2f",
                                        estPose.getTranslation().getX(),
                                        estPose.getTranslation().getY(),
                                        estPose.getRotation().getDegrees()));
                                if (useVision) {
                                    SmartDashboard.putBoolean("Ovr Conf", operatorOverrideConfidence);
                                    if ( runningTrapPath || operatorOverrideConfidence) {
                                        estStdDevs = kTrapStdDevs;
                                    }
                                    m_driveSubsystem.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                                    lastEstTimestampFront = Timer.getFPGATimestamp();
                                }

                            });
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            if ((photonEstimatorBack != null)&& !runningTrapPath && !isZoomCameraReadingValid) {
                // Correct pose estimate with vision measurements
                try {
                    var visionEstBack = getEstimatedGlobalPoseBack();
                    visionEstBack.ifPresent(
                            est -> {
                                var estPose = est.estimatedPose.toPose2d();
                                // Change our trust in the measurement based on the tags we can see
                                var estStdDevs = getEstimationStdDevs(cameraBack, estPose, photonEstimatorBack);
                                field2d.getObject("MyRobot" + cameraBack.getName()).setPose(estPose);
                                SmartDashboard.putString("Vision pose", String.format("(%.2f, %.2f) %.2f",
                                        estPose.getTranslation().getX(),
                                        estPose.getTranslation().getY(),
                                        estPose.getRotation().getDegrees()));
                                if (useVision) {
                                    m_driveSubsystem.addVisionMeasurement(est.estimatedPose.toPose2d(),
                                            est.timestampSeconds, estStdDevs);
                                    lastEstTimestampBack = Timer.getFPGATimestamp();
                                } else {
                                    visionPose = est.estimatedPose.toPose2d();
                                }
                            });
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            double curTime = Timer.getFPGATimestamp();
            // if both cameras are stale set warning
            if (((curTime - lastEstTimestampFront) > kTargetConfidenceDelta) && 
                ((curTime - lastEstTimestampBack) > kTargetConfidenceDelta)) {
                // System.out.println("false: " + targetConf);
                confidence = false;
                SmartDashboard.putBoolean("Target Conf", false);
            } else {
                // System.out.println("true: " + targetConf);
                confidence = true;
                SmartDashboard.putBoolean("Target Conf", true);
            }
        }

       // field2d.setRobotPose(getCurrentPose());
    }

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
        Pose2d target = Robot.isRedAlliance()? redGoal: blueGoal;
        Pose2d robot = getAdjustedRobotPose();
        double headingToTarget = Math.atan((target.getY() - robot.getY())/(robot.getX() - target.getX()));
        SmartDashboard.putNumber("HeadingToTarget", headingToTarget);
      //  System.out.println("getting heading");
        return new Rotation2d(-headingToTarget);
    }

    public double getDistanceToSpeakerInMeters() {
        Pose2d target = Robot.isRedAlliance()? redGoal: blueGoal;
        Pose2d robot = getAdjustedRobotPose();
        double distance = PhotonUtils.getDistanceToPose(target, robot);
      //  SmartDashboard.putNumber("DistanceToTarget", distance);
        return distance;
    }

    public double getStaticDistanceToSpeakerInMeters() {
        Pose2d target = Robot.isRedAlliance()? redGoal: blueGoal;
        Pose2d robot = m_driveSubsystem.getState().Pose;
        double distance = PhotonUtils.getDistanceToPose(target, robot);
        SmartDashboard.putNumber("DistanceToTarget", distance);
        return distance;
    }
    public double getDistanceToTargetForAuto(){
        // double robotXSpeed = m_driveSubsystem.getXVelocity();
        // double robotYSpeed = m_driveSubsystem.getYVelocity();
        // double robotXAcceleration = -getAccelerationY();
        // double robotYAcceleration = getAccelerationX();

        Pose2d target = Robot.isRedAlliance()? redGoal: blueGoal;
        
        // double visionDelay = 0.5;
        // Transform2d displacement = new Transform2d((robotXSpeed*visionDelay + 0.5*robotXAcceleration*visionDelay*visionDelay), (robotYSpeed*visionDelay +  0.5*robotYAcceleration*visionDelay*visionDelay), new Rotation2d());
        Pose2d adjustedRobotPose = visionPose;

        double distance = PhotonUtils.getDistanceToPose(target, adjustedRobotPose);
        return distance;
    }

    public double getAccelerationX() {
        return (mypigeon.getAccelerationX().getValueAsDouble())*9.81;
    }
    public double getAccelerationY() {
        return (mypigeon.getAccelerationY().getValueAsDouble())*9.81;
    }

     public Pose2d getAdjustedRobotPose() {
        double robotXSpeed = m_driveSubsystem.getXVelocity();
        double robotYSpeed = m_driveSubsystem.getYVelocity();
        double robotXAcceleration = -getAccelerationY();
        double robotYAcceleration = getAccelerationX();
        Pose2d robot = new Pose2d(m_driveSubsystem.getState().Pose.getX(), m_driveSubsystem.getState().Pose.getY(), m_driveSubsystem.getState().Pose.getRotation());
         
        double shotTime = (getStaticDistanceToSpeakerInMeters() / (3.81 * 2 * Math.PI)) * shootOnMoveFudgeFactor; //speed of shot in m/s

        SmartDashboard.putNumber("ShootOnMove Fudge Factor: ", shootOnMoveFudgeFactor);
       
        Transform2d adjustment = new Transform2d(robotXSpeed*shotTime + 0.5*robotXAcceleration*shotTime*shotTime, robotYSpeed*shotTime +  0.5*robotYAcceleration*shotTime*shotTime, new Rotation2d());
       
        SmartDashboard.putNumber("x adjustment", robotXSpeed*shotTime + 0.5*robotXAcceleration*shotTime*shotTime);
        SmartDashboard.putNumber("y adjustment", robotYSpeed*shotTime + 0.5*robotYAcceleration*shotTime*shotTime);

        if (Robot.isRedAlliance()) {
            robot.rotateBy(new Rotation2d(Math.toRadians(180)));
        }

        SmartDashboard.putNumber("RobotXSpeed", robotXSpeed);
        SmartDashboard.putNumber("RobotYSpeed", robotYSpeed);
        SmartDashboard.putNumber("RobotXAcceleration", robotXAcceleration);
        SmartDashboard.putNumber("RobotYAcceleration", robotYAcceleration);

        robot.rotateBy(m_driveSubsystem.getState().Pose.getRotation());

        Pose2d adjustedRobotPose = robot.plus(adjustment);

       // field2d.getObject("MyRobotAdjusted").setPose(adjustedRobotPose);

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

    public void useVision(boolean useCameraVision) {
        useVision = useCameraVision;
    }

    public void stopDrivingToTrap() {
        runningTrapPath = false;
    }

    public void drivingToTrap() {
        runningTrapPath = true;
    }

    public boolean haveGoodVisionLock() {
       return (Timer.getFPGATimestamp() - lastEstTimestampFront) < 0.2;
    }

    public void setConfidence(boolean confidence) {
        this.operatorOverrideConfidence = confidence;
    }

 
}
