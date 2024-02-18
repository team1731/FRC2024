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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Robot;

import java.util.HashMap;
import java.util.Optional;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase implements ToggleableSubsystem {
    private HashMap<String, CameraTransform> cameraMap;
    private HashMap<String, PhotonPoseEstimator> estimatorMap;
    private final PhotonCamera cameraFront;
    private final PhotonCamera cameraBack;

    private PhotonPoseEstimator photonEstimatorFront;
    private PhotonPoseEstimator photonEstimatorBack;
    private final Field2d field2d = new Field2d();

    private CommandSwerveDrivetrain driveSubsystem;
    private double lastEstTimestamp = 0;

    private boolean enabled;
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    private boolean initialized;
    public boolean isInitialized() {
        return initialized;
    }

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    public VisionSubsystem(boolean enabled, CommandSwerveDrivetrain driveSubsystem) {
        this.enabled = enabled;
        this.driveSubsystem = driveSubsystem;
        cameraFront = new PhotonCamera(kCameraNameFront);
        cameraBack = new PhotonCamera(kCameraNameBack);
        cameraMap = new HashMap<String, CameraTransform>();
        estimatorMap = new HashMap<String, PhotonPoseEstimator>();

        if (cameraFront != null) {
            System.out.println("VisionSubsystem: Adding vision measurement from " + kCameraNameFront);
            System.out.println(
                "VisionSubsystem: Adding camera " + kCameraNameFront + "!!!!!!! " + cameraFront);
            CameraTransform transform = new CameraTransform(cameraFront, kRobotToCamFront);
            System.out.println("Camera in transform = " + transform.camera);
            System.out.println("Putting cameraTranform into map key=" + kCameraNameFront + ", " + transform);
            this.cameraMap.put(kCameraNameFront, transform);
             photonEstimatorFront =
                new PhotonPoseEstimator(
                        kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraFront, kRobotToCamFront);
            this.estimatorMap.put(kCameraNameFront, photonEstimatorFront);
            photonEstimatorFront.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
          }
        if (cameraBack != null) {
            System.out.println("VisionSubsystem: Adding vision measurement from " + kCameraNameBack);
            System.out.println(
                "VisionSubsystem: Adding camera " + kCameraNameBack + "!!!!!!! " + cameraBack);
            CameraTransform transform = new CameraTransform(cameraBack, kRobotToCamBack);
            System.out.println("Camera in transform = " + transform.camera);
            System.out.println("Putting cameraTranform into map key=" + kCameraNameBack + ", " + transform);
            this.cameraMap.put(kCameraNameBack, transform);
             photonEstimatorBack =
                new PhotonPoseEstimator(
                        kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraBack, kRobotToCamBack);
            this.estimatorMap.put(kCameraNameBack, photonEstimatorBack);
            photonEstimatorBack.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
          }
       
        // write initial values to dashboard
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        tab.addString("Pose (X, Y)", this::getFormattedPose).withPosition(0, 4);
        tab.addNumber("Pose Degrees", () -> getCurrentPose().getRotation().getDegrees()).withPosition(1, 4);
        tab.add(field2d);

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(kTagLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, kRobotToCamBack);

            cameraSim.enableDrawWireframe(true);
        }

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // get the subtable called "photonvision"
        NetworkTable photonVisionTable = inst.getTable("photonvision/USB_Camera");
        if (photonVisionTable.containsKey("hasTarget")) { // .getEntry("hasTarget").exists()){
            initialized = true;
            System.out.println("VisionSubsystem: Initializing Success !!!!!!");
        } else {
            System.out.println("VisionSubsystem: Initializing Failed: " + " Keys: " + photonVisionTable.getKeys().toString());
        }
    }

  private String getFormattedPose() {
    if (enabled){
      var pose = getCurrentPose();
      return String.format("(%.2f, %.2f)", 
          Units.metersToInches(pose.getX()), 
          Units.metersToInches(pose.getY()));
    }
    else{
      return null;
    }
  }

  public Pose2d getCurrentPose() {
    if (enabled){
      return driveSubsystem.getState().Pose;
    }
    else{
      return null;
    }
  }

    @Override
    public void periodic() {
        if (enabled && initialized) {
            System.out.println("Map Size: " + cameraMap.size() + "Est Map Size: " + estimatorMap.size());

            for (String cameraName : this.cameraMap.keySet()) {
                System.out.println("VisionSubsystem: getting tranform out of map key= " + cameraName);
                CameraTransform transform = this.cameraMap.get(cameraName);
                System.out.println("VisionSubsystem: transform: " + transform.toString());
                PhotonCamera camera = transform.camera;
                if (camera != null) {
                    PhotonPoseEstimator cameraEstimator = this.estimatorMap.get(cameraName);
                    System.out.println("VisionSubsystem: cameraEstimator " + cameraEstimator.toString());
                    try {

                        // Correct pose estimate with vision measurements
                        var visionEst = getEstimatedGlobalPose(camera, cameraEstimator);
                        System.out.println("VisionSubsystem: visionEst " + visionEst.toString());
                        visionEst.ifPresent(
                                est -> {
                                    var estPose = est.estimatedPose.toPose2d();
                                    // Change our trust in the measurement based on the tags we can see
                                    var estStdDevs = getEstimationStdDevs(camera, estPose, cameraEstimator);

                                    System.out.println("VisionSubsystem: Adding vision measurement from " + cameraName);
                                    Pose2d cameraPose2d = est.estimatedPose.toPose2d();
                                    field2d.getObject("MyRobot" + cameraName).setPose(cameraPose2d);
                                    SmartDashboard.putString("Vision pose", String.format("(%.2f, %.2f) %.2f",
                                        cameraPose2d.getTranslation().getX(),
                                        cameraPose2d.getTranslation().getY(),
                                        cameraPose2d.getRotation().getDegrees()));

                                    driveSubsystem.addVisionMeasurement(
                                            est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                                });

                        // // Apply a random offset to pose estimator to test vision correction
                        // if (controller.getBButtonPressed()) {
                        // var trf =
                        // new Transform2d(
                        // new Translation2d(rand.nextDouble() * 4 - 2, rand.nextDouble() * 4 - 2),
                        // new Rotation2d(rand.nextDouble() * 2 * Math.PI));
                        // driveSubsystem.resetPose(driveSubsystem.getPose().plus(trf), false);
                        // }

                        // Log values to the dashboard
                        // driveSubsystem.log();

                    } catch (Exception e) {
                        System.out.println("Error: PhotonVision Camera not connected !!!: " + camera.getName());
                    }
                } else {
                    System.out.println("Error: Camera Object is Null: " + cameraName);
                }
            }
            enabled = false;
        }
    }

    public PhotonPipelineResult getLatestResult(PhotonCamera camera) {

        PhotonPipelineResult cameraResult = camera.getLatestResult();
        return cameraResult;
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonCamera camera, PhotonPoseEstimator photonEstimator) {
        var visionEst = photonEstimator.update();
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        // if (Robot.isSimulation()) {
        //     visionEst.ifPresentOrElse(
        //             est ->
        //                     getSimDebugField()
        //                             .getObject("VisionEstimation")
        //                             .setPose(est.estimatedPose.toPose2d()),
        //             () -> {
        //                 if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
        //             });
        // }
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(PhotonCamera camera, Pose2d estimatedPose, PhotonPoseEstimator photonEstimator) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = getLatestResult(camera).getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    // ----- Simulation

    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }
}
