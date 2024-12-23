package frc.robot;

import java.util.Map;

// import com.ctre.phoenix6.configs.FeedbackConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.util.Gains;
import frc.robot.util.log.LogWriter.Log;
import frc.robot.util.log.LogWriter.LogMode;

public final class Constants {
    public static final double stickDeadband = 0.1;
	public static final int kTICKS = 33024; // 16.125 * 2048;
    public static final String CANBUS_NAME = "canivore1";

    public static enum GamePiece {
        CONE, CUBE
    }

    public static enum HighPickup {
        FEEDER, SHELF
    }

    public static final class LogConstants {
        /*
         * To write to a log you must:
         * 1. Set loggingEnabled = true
         * 2. Set the desired logMode (CSV, DATA_LOG)
         * 2. Set the desired loggers below = true
         */
        public static final boolean loggingEnabled =false;    // note: must also turn on applicable loggers below
        public static final boolean logNetworkTables = false;   // only applicable when logMode = DATA_LOG
        public static final LogMode logMode = LogMode.CSV;

        // list of loggers and enabled status, note: must also enable logging above
        public static final Map<Log, Boolean> loggers = Map.of(
            Log.MESSAGE, false,
            Log.ARM_PATH_RECORDING, false,
            Log.POSE_ESTIMATIONS, false
        );

        // Arm path recording constants
        public final static double recordingPeriod = 0.01; // seconds
        public final static double recordingOffset = 0.005; // seconds 
    }

    public static final class FieldConstants {
        // Note: Field dimensions and April Tag positions pulled from the 2023 Field and Layout Marking document
        public static final double kFieldLength = Units.inchesToMeters(651.22);
        public static final double kFieldWidth = Units.inchesToMeters(315.1);

        public static class AprilTagPoseValues {
            public int id;
            public double x;
            public double y;
            public double z;
            public double yaw;

            public AprilTagPoseValues(int tagId, double xInches, double yInches, double zInches, double yawDegrees) {
                id = tagId;
                x = Units.inchesToMeters(xInches);
                y = Units.inchesToMeters(yInches);
                z = Units.inchesToMeters(zInches);
                yaw = Units.degreesToRadians(yawDegrees);
            }

            public AprilTag getAprilTag() {
                return new AprilTag(id, new Pose3d(x, y, z, new Rotation3d(0.0, 0.0, yaw)));
            }
        }

        public static final AprilTagPoseValues kAprilTagPose1 = new AprilTagPoseValues(1, 610.77, 42.19, 18.22, 180);
        public static final AprilTagPoseValues kAprilTagPose2 = new AprilTagPoseValues(2, 610.77, 108.19, 18.22, 180);
        public static final AprilTagPoseValues kAprilTagPose3 = new AprilTagPoseValues(3, 610.77, 174.19, 18.22, 180);
        public static final AprilTagPoseValues kAprilTagPose4 = new AprilTagPoseValues(4, 636.96, 265.74, 27.38, 180);
        public static final AprilTagPoseValues kAprilTagPose5 = new AprilTagPoseValues(5, 14.25, 265.74, 27.38, 0);
        public static final AprilTagPoseValues kAprilTagPose6 = new AprilTagPoseValues(6, 40.45, 174.19, 18.22, 0);
        public static final AprilTagPoseValues kAprilTagPose7 = new AprilTagPoseValues(7, 40.45, 108.19, 18.22, 0);
        public static final AprilTagPoseValues kAprilTagPose8 = new AprilTagPoseValues(8, 40.45, 42.19, 18.22, 0);
    }

    //Operator controls List

    //AXES = JOYSTICKS/BACK WHEEL CONTROLS
    /*
    getRawAxis(0) = left joystick(L/R): 
    getRawAxis(1) = left joystick(U/D): 
    getRawAxis(2) = left back wheel:
    getRawAxis(3) = right joystick(L/R): 
    getRawAxis(4) = right joystick(U/D): 
    getRawAxis(5) = right back wheel: 

    //BUTTONS
    Button(1) = top Left back toggle:                                   (Toggled towards Driver);
    Button(2) = top Left front toggle:                                  (Toggled away from Driver)
    Button(3) = top Left front toggle:                                  (Toggled towards Driver)
    Button(4) = front Left front toggle(L):                             (Toggled away from Driver)
    Button(5) = front Left front toggle(L):                             (Toggled towards Driver)
    Button(6) = front Left front toggle(R):                             (Toggled away from Driver)
    Button(7) = front Left front toggle(R):                             (Toggled towards Driver)
    Button(8) = front Right front toggle(R):                            (Toggled towards Driver)
    Button(9) = front Right front toggle(R):                            (Toggled away from Driver)
    Button(10) = top Right front toggle:                                (Toggled towards Driver)
    Button(11) = top Right front toggle:                                (Toggled away from Driver)
    Button(12) = top Right back toggle:                                 (Toggled towards Driver)
    Button(14) = Front Left Bottom(1):                                  (Top Button)
    Button(15) = front Left Bottom(2):                                  (Bottom Button)
    Button(16) = front Right Bottom:                                    (Clicking the scrollwheel)
    */

    public static final class OperatorConsoleConstants {
        // Buttons
        public static int kPreventScoreBtnId = 13;
        public static int kReleaseBtnId = 14;
        public static int kIntakeBtnId = 15;
        public static int kWheelLockBtnId = 16;

        // Sticks
        public static int kProximalAxisId = 1;
        public static int kDistalAxisId = 4;
        public static int kxAxis = 0;

        // Switches - start at 1 not 0
        public static int kShooterSwitch = 1;
        public static int kWristAngle3 = 2;
        public static int kWristAngle4 = 3;
        public static int kWristSwitchId = 4;

        public static int kScoreMediumSwitchId = 8;
        public static int kScoreHighSwitchId = 9;
        public static int kAutoRecoverySwitchId = 10;
        public static int kKillSwitchId = 11;

        public static int kElevatorControlButton = 12;
    }

    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double wheelWidth = Units.inchesToMeters((25.0/2.0) - 2.5);
        public static final double wheelFront = Units.inchesToMeters((27.0/2.0) - 5.0 - 2.5);
        public static final double wheelBack  = Units.inchesToMeters((27.0/2.0)- 2.5);
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.75 / 1.0); //6.86:1
        public static final double angleGearRatio = (150.0/ 7.0); //12.8:1  

        // if center of robot is (0,0) then Positive X moves to front and Positive Y moves toward left
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelFront / 2.0, wheelWidth),    // front left
                new Translation2d(wheelFront / 2.0, -wheelWidth),   // front right
                new Translation2d(-wheelBack / 2.0, wheelWidth),    // back  left
                new Translation2d(-wheelBack / 2.0, -wheelWidth));  // back  right

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.3; // was .6
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;  // these values are suspect ?????  ratttly swerve noise is from this but not sure
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.18576/ 12); //divide by 12 to convert from volts to percent output for CTRE  was .667/12
        public static final double driveKV = (2.3317 / 12);
        public static final double driveKA = (0.25916 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 5.0; // disabled for testing = 3.0; //meters per second
        public static final double maxAngularVelocity = 6.0; // disabled for testing = 2.7;

        /* Neutral Modes */

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 1;
            public static final double angleOffset = 230.01;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 2;
            public static final double angleOffset = 332.578;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 3;
            public static final double angleOffset = 133.242;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 4;
            public static final double angleOffset = 64.336;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class AutoConstants {
        public static final String kAutoDefault = "5_JustDrive"; //"Amp_1";
    	public static final String kAutoCodeKey = "Auto Selector";

        public static final double kMaxSpeedMetersPerSecond = 0.5; // disabled for testing = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5; // disabled for testing = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 3.0;
        public static final double kPYController = 3.0;
        public static final double kPThetaController = 2;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }
      
    public static final class StateConstants {
        public enum ResultCode {
            SUCCESS, FAILED, INVALID_REQUEST,
            // Arm Specific
            ARM_BUFFERING_FAILED, ARM_MOTION_START_FAILED
        }
        
    }

    public static final class ArmStateConstants {       
        public final static double coneFlipFlexPosition = 0.51;
        public final static double wristOnlyFlexMaxVelocity = 2000;
        public final static double autoScoreConeDelay = 0.5;
    }

    public static final class ElevatorConstants {
        public final static int elevatorCanId1 = 21;
        public final static int elevatorCanId2 = 22;

        public final static double MMVel = 2; // 5 rotations per second cruise
        public final static double MMAcc = 5; // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel 
        public final static double MMJerk = 50;
    
        // initialze PID controller and encoder objects
        public final static double kP = 60;
        public final static double kI = 0;
        public final static double kD = 0.1;
        public final static double kV = 0.12;
        public final static double kS = 0.25; // Approximately 0.25V to get the mechanism moving
    
        public final static double StM_Ratio = 1;

        // public static final int ELEVATOR_CURRENT_LIMIT_A = 18;
        // public static final int ELEVATOR_HOLD_CURRENT_LIMIT_A = 5;
        // public static final int EJECT_CURRENT_LIMIT = 20;

        // public final static double elevatorSpeed = 0.25;

        // public final static double elevatorStartedVelocityThreshold = 1000;
        // public final static double elevatorHoldingVelocityThreshold = 60;

        // public static final double ELEVATOR_HOLD_POWER = 0.07;

        // // PID coefficients
        // public static final double kP = 5e-5; 
        // public static final double kI = 1e-6;
        // public static final double kD = 0; 
        // public static final double kIz = 0; 
        // public static final double kFF = 0.000156; 
        // public static final double kMaxOutput = 1; 
        // public static final double kMinOutput = -1;
        // public static final double maxRPM = 5700;

        // // Smart Motion Coefficients
        // public static final double minVel = 500; // rpm
        // public static final double maxVel = 2000; // rpm
        // public static final double maxAcc = 1500;
        // public static final double allowedErr = 1;

        // public static final int smartMotionSlot = 0;

        // Motor Direction
        public final static InvertedValue elevatorDirection = InvertedValue.Clockwise_Positive; // or Clockwise_Positive

        // Positions
        public final static double elevatorHomePosition = 0;
        public final static double elevatorExtendedPosition = 75;
        public final static double elevatorPositionTolerance = 0.05;
        public static final double wristClearsPosition = 0;
        public static final double elevatorAmpPosition = 75;
        public static final double elevatorShooterAsIntakePosition = 20;
        public static final double elevatorAmpReversePosition = 10;
        public static double elevatorTrapPosition = 78.5;  

    }

    public static final class IntakeConstants {
        public final static int intakeCancoderId = 11;
        public final static int feederCancoderId = 12;
        /*
         ************************************************************************************************
         * THESE VALUES NEED TO CHANGE IF WE WORK ON THE Intake Subsystem !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
         *   Motors: Intake & Feeder
         * **********************************************************************************************
         */
        public static final int INTAKE_CURRENT_LIMIT_A = 40;
        public static final int FEEDER_CURRENT_LIMIT_A = 40;
        // public static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;
        // public static final int EJECT_CURRENT_LIMIT = 20;

        public final static double intakeSpeed = 1;
        public final static double feederSpeed = 1;
        public final static double trapFeedSpeed = .5;

        public final static double intakeStartedVelocityThreshold = 1000;
        public final static double intakeHoldingVelocityThreshold = 60;

        public static final double INTAKE_HOLD_POWER = 0.07;
    }

    public static final class WristConstants {
        public final static int wrist1CanId = 14;
        public final static int wrist2CanId = 13;
        public final static InvertedValue wrist1Direction = InvertedValue.Clockwise_Positive; // or Clockwise_Positive
        public final static InvertedValue wrist2Direction = InvertedValue.CounterClockwise_Positive; // or Clockwise_Positive
        /*
         ************************************************************************************************
         * THESE VALUES NEED TO CHANGE IF WE WORK ON THE Intake Subsystem !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
         *   Motors: Intake & Feeder
         * **********************************************************************************************
         */

        public final static double MMVel        = 80; // 5 rotations per second cruise
        public final static double MMVelSlow    = 25;
        public final static double MMAcc        = 150; // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel 
        public final static double MMJerk       = 0;
    
        // initialze PID controller and encoder objects
        public final static double kP = 60;
        public final static double kI = 0;
        public final static double kD = 0.1;
        public final static double kV = 0.12;
        public final static double kS = 0.25; // Approximately 0.25V to get the mechanism moving

        // Positions
        public final static double wristHomePosition = 0;
        public final static double wristAmpPosition = 28*0.6;
        public final static double wristAmpReversePosition = 91*0.6;
        public final static double wristNewTrapPosition = 60*0.6;
        public final static double wristTrapPosition = 59*0.6;
        public static final double IntakePosition = 0;
    }

    public static final class ShooterConstants{
        public final static int shooterCancoderId1 = 15;
        public final static int shooterCancoderId2 = 16;
        public static final int SHOOTER_CURRENT_LIMIT_A = 20;

        // PID coefficients
        public static final double kP = 8e-4; 
        public static final double kI = 0; //1e-6;
        public static final double kD = 0; 
        public static final double kIz = 0; 
        public static final double kFF = 0.000356; 
        public static final double kMaxOutput = 1.0; 
        public static final double kMinOutput = -1.0;
        public static final double maxRPM = 5700;

        public final static double kMotorSpeed1 = 1;
        public final static double kMotorSpeed2 = 1;
    }

    public static final class ArmConstants {
        /*
         ************************************************************************************************
         * THESE VALUES NEED TO CHANGE IF WE WORK ON THE ARM!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
         *   the better proximal should be 2443    Auto starting absolute numbers at start of houston 2436   1310
         * **********************************************************************************************
         */
         public final static int distalAbsoluteTicsOffset = 15;
         public final static int proximalAbsoluteTicsCenter = 2732;   // 2660 2664   2652 
         public final static int distalAbsoluteTicsCenter = 1009 + distalAbsoluteTicsOffset; // 1105 1106  was 1113

        /*
         ************************************************************************************************
         * These are absolute values based off the center tics above
         * **********************************************************************************************
         */

         public final static int proximalEstimatedAutoAbsolute = proximalAbsoluteTicsCenter - 205; //  2565; // only used if we are not getting reasonable values from absolute encoder
         public final static int distalEstimatedAutoAbsolute = distalAbsoluteTicsCenter + 206; //1926; // only used if we are not getting reasonable values from absolute encoder

                 // bounds checking for absolute encoder values, work around for spotty values we are receiving
        // these first set of bounds are used when we don't know where the arm is and we mainly want to make sure that
        // we are discarding readings that are obviously absurd
        public final static int[] proximalAbsoluteBounds = new int[] {proximalAbsoluteTicsCenter - 714,  proximalAbsoluteTicsCenter + 1286};  // 2000 4000
        public final static int[] distalAbsoluteBounds = new int[] {distalAbsoluteTicsCenter - 790, distalAbsoluteTicsCenter + 710}; // 1000, 2500
        // these second set of bounds are used when we are starting from auto where we do know about where we are
        // we want to keep the range checking narrower for this situation
        public final static int[] proximalAbsoluteBoundsAuto = new int[] {proximalEstimatedAutoAbsolute - 150, proximalEstimatedAutoAbsolute + 150};  // {2430, 2585}
        public final static int[] distalAbsoluteBoundsAuto = new int[] {distalEstimatedAutoAbsolute - 100, distalEstimatedAutoAbsolute + 100};   //  {1925, 2060}

        public final static int proximalCancoderId = 11;
        public final static int distalCancoderId = 10;
        public final static int wristCancoderId = 12;
        public final static int intakeCancoderId = 13;
        public final static int shooterCancoderId = 2;
        public final static double proximalRelativeTicsPerAbsoluteTick = 135;  // theoretically this should be 140
        public final static double distalRelativeTicsPerAbsoluteTick = 90;
        public final static int pointDurationMS = 10;
        public final static int minBufferedPoints = 10;
        public final static double proximalHomePosition = -4388;
        public final static double distalHomePosition = 10300;
        public final static double wristHomePosition = 0.74;
        public final static double intakeStartedVelocityThreshold = 1000;
        public final static double intakeHoldingVelocityThreshold = 60;

        public final static double wristResetPostionThreshold = 0.2;
        public final static double distalMaxAdjustmentTicks = 9233;
        public final static double wristMaxAdjustment = 0.05;
        public final static double emergencyModeMaxArmVelocity = 2000; // max for Falcon motors is 6800 velocity units
        public final static double mostlyExtendedThreshold = 0.5; // percentage of the path completed to consider mostly extended
        public final static double proximalOutOfPositionThreshold = -37500;
        
        public final static double onDemandIntakeSpeed = 0.8; // used when intake button is pressed on operator controller
        public final static double downedConeIntakeSpeed = 0.75;
        public final static double coneIntakeSpeed = 0.75;
        public final static double cubeIntakeSpeed = -0.7;

        // Arm PID constants
        public final static int armPIDLoopIdx = 0;

        // Wrist PID coefficients
        public final static double wristP = 8e-4;
        public final static double wristI = 0;
        public final static double wristD = 0; 
        public final static double wristIz = 0; 
        public final static double wristFF = 0.000356; //.000156
        public final static double wristMaxOutput = 1.0; 
        public final static double wristMinOutput = -1.0;
        public final static double wristMaxRPM = 5700;

        // Wrist limits
        public static final int WRIST_CURRENT_LIMIT = 24;

        // Wrist Smart Motion Coefficients
        public final static double wristMaxVel = 2000;
        public final static double wristMinVel = 0;
        public final static double wristMaxAcc = 1500; 
        public final static double wristAllowedErr = 0;

         // Hand limits
        public static final int INTAKE_CURRENT_LIMIT_A = 40;
        public static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;
        public static final int EJECT_CURRENT_LIMIT = 20;
        public static final double INTAKE_OUTPUT_POWER = 1.0;
        public static final double INTAKE_HOLD_POWER = 0.07; 


        //Geometry Constants
        public final static double proximalArmLength = 35.75; //inches
        public final static double distalArmLength = 32; //inches
        public final static double proximalTicksPerDegree = 286720.0/360.0;
        public final static double distalTicksPerDegree = 512.0; 
        public final static double ThrottleAtFullExtensionDistalAndProximal = 0.02639; //TBD empirically
        public final static double ThrottleAtFullExtensionDistal = .03225; //TBD empirically
        public final static double distalFullExtensionDistance =33.0; //TBD empirically
        public final static double armFullExtensionDistance = 56.25;
        
        /**
         * How many sensor units per rotation.
         * Using Talon FX Integrated Sensor.
         * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
         */
        public final static int kSensorUnitsPerRotation = 2048;
        
        /**
         * Motor neutral dead-band, set to the minimum 0.1%.
         */
        public final static double kNeutralDeadband = 0.001;
        
        /**
         * PID Gains may have to be adjusted based on the responsiveness of control loop
         * 	                                    			  kP   kI    kD     kF             Iz    PeakOut */
        public final static Gains kGains_MotProf = new Gains( 0.25, 0.0,  0.0, 1023.0/15200.0,  400,  1.00 ); /* measured 6800 velocity units at full motor output */
        public final static int kPrimaryPIDSlot = 0; // any slot [0,3]
    }

    public static class Vision {
        public static final double kTargetConfidenceDelta = 0.5;

        public static final String kCameraNameFront = "Global_Shutter_Camera";
        public static final String kCameraNameBack = "ArducamUSB1";
        public static final double kMaxDistanceBetweenPoseEstimations = 1.0;

        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCamFront =
                new Transform3d(new Translation3d(-0.336555,  -0.054, 0.4572), new Rotation3d(0, -Units.degreesToRadians(12), Units.degreesToRadians(180)));
        public static final Transform3d kRobotToCamBack =
                new Transform3d(new Translation3d(-0.336555,0, 0.4572), new Rotation3d(0, -Units.degreesToRadians(30.5), Units.degreesToRadians(180)));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.05, 0.05, 1);
        public static final Matrix<N3, N1> kTrapStdDevs = VecBuilder.fill(0.1, 0.1, 0.001);
        public static final Matrix<N3, N1> kVSLAMStdDevs = VecBuilder.fill(0.0001, 0.0001, 0.0001);
    }

    public static final class OpConstants{
        // KEYBOARD CONSTANTS   
        public static final int kPWM_LedString = 1;     // PWM # for Addressable Led String
        public static final int kLedStringLength = 11;  // Length of Addressable Led String; 33 individual / sets of 3
        public static final double kLedStringBlinkDelay = 0.1;  // Delay in Seconds of Addressable Led String

        public enum LedOption {
            INIT, YELLOW, PURPLE, BLACK, WHITE, BLUE, RED, GREEN
          }
    }

    // public static final class VisionConstants {
	// 	// Ensure measurements are in METERS
	// 	public static final double kMaxDistanceBetweenPoseEstimations = 1.0;

    //     // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much you trust your various sensors. 
    //     // Smaller numbers will cause the filter to "trust" the estimate from that particular component more than the 
    //     // others. This in turn means the particular component will have a stronger influence on the final pose estimate.
    //     public static final Matrix<N3, N1> kVisionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(180));

    //     public static class CameraMountPoseValues {
    //         public String id;
    //         public double x;
    //         public double y;
    //         public double z;
    //         public double yaw;
    //         public double pitch;

    //         // Note: this constructor assumes the camera is mounted parallel to the floor
    //         public CameraMountPoseValues(String cameraId, double xInches, double yInches, double zInches, double yawDegrees, double pitchDegrees) {
    //             id = cameraId;
    //             x = Units.inchesToMeters(xInches);
    //             y = Units.inchesToMeters(yInches);
    //             z = Units.inchesToMeters(zInches);
    //             yaw = Units.degreesToRadians(yawDegrees);
    //             pitch = Units.degreesToRadians(pitchDegrees);
    //         }

    //         public Transform3d getPoseTransform() {
    //             return new Transform3d(new Translation3d(x, y, z), new Rotation3d(0.0,pitch,yaw));
    //         }
    //     }

    //     public static final String kCameraMount1Id = "leftcamera";  //camera on the left looking back
    //     public static final String kCameraMount2Id = "Global_Shutter_Camera";  // camera on the right looking back
    //     public static final String kCameraMount3Id = "USB_Camera2";  // camera on the arm
    //     public static final CameraMountPoseValues kCameraMount1Pose = new CameraMountPoseValues(kCameraMount1Id, -5.3,10.253, 17.0, 135.0,0.0);
    //     public static final CameraMountPoseValues kCameraMount2Pose = new CameraMountPoseValues(kCameraMount2Id, -5.3, -10.253, 17.0, 225.0,0.0);
    //     public static final CameraMountPoseValues kCameraMount3Pose = new CameraMountPoseValues(kCameraMount3Id, -11, -5.0, 37, 0.0,-80.0);

	// 	// #region TurnPID
	// 	public static final double kTurnP = 0.05;
	// 	public static final double kTurnI = 0;
	// 	public static final double kTurnD = 0.00;
	// //	public static final double kMaxTurnVelocity = 360;
	// //	public static final double kMaxTurnAcceleration = 360;
	// 	public static final double kTurnToleranceDeg = 5;
	// 	public static final double kTurnRateToleranceDegPerS = 10; // degrees per second
	// 	// #endregion
    // }

}
