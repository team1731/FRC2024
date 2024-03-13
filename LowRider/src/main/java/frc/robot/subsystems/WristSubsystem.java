package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants.WristConstants;
import frc.robot.TunerConstants;
import frc.robot.util.LinearServo;

public class WristSubsystem extends SubsystemBase implements ToggleableSubsystem {

    // motors for the Wrist
    private TalonFX wristMotor1;
    private TalonFX wristMotor2;
    private double desiredPosition;
    private double arbitraryFeedForward = 0.0;
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;


    private DynamicMotionMagicVoltage mmReq = new DynamicMotionMagicVoltage(
        0, 
        WristConstants.MMVel, 
        WristConstants.MMAcc, 
        WristConstants.MMJerk

        
    );

    // private MotionMagicVoltage mmReq1 = new MotionMagicVoltage(0);

    private Servo trapFlapServo = new LinearServo(0, 50,20 );

    private CommandSwerveDrivetrain driveSubsystem;
    private VisionSubsystem m_visionSubsystem;

    private boolean enabled;

    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public WristSubsystem(boolean enabled, CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem) {
        this.enabled = enabled;
        if(enabled){
            initializeWristMotors();
        }
        this.driveSubsystem = drivetrain;
        this.m_visionSubsystem = visionSubsystem;
    }

    /*
     * Wrist MOTOR MOVEMENT
    */

    public void moveWrist(double position) {
        if(!enabled) return;
        mmReq.withVelocity(WristConstants.MMVel);
        wristMotor1.setControl(mmReq.withPosition(position).withFeedForward(arbitraryFeedForward));
        wristMotor2.setControl(mmReq.withPosition(position).withFeedForward(arbitraryFeedForward));
        System.out.println("Moving wrist to" + position);
        desiredPosition = position;
    }

    public void moveWristSlow(double position, double velocity) {
        if(!enabled) return;
        mmReq.withVelocity(velocity);
        wristMotor1.setControl(mmReq.withPosition(position).withFeedForward(arbitraryFeedForward));
        wristMotor2.setControl(mmReq.withPosition(position).withFeedForward(arbitraryFeedForward));
        System.out.println("Moving wrist slow to" + position);

        desiredPosition = position;
    }

    // Initialize Motors
    private void initializeWristMotors() {
        if (!enabled) {
            return;
        }
        System.out.println("WristSubsystem: Starting UP & Initializing Wrist motors !!!!!!");

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        //Set Defaults
        
        wristMotor1 = new TalonFX(WristConstants.wrist1CanId, "canivore1");
        wristMotor2 = new TalonFX(WristConstants.wrist2CanId, "canivore1");
        wristMotor1.getConfigurator().apply(cfg);
        wristMotor2.getConfigurator().apply(cfg);

        wristMotor1.setNeutralMode(NeutralModeValue.Brake);
        wristMotor2.setNeutralMode(NeutralModeValue.Brake);

        /* Configure current limits */
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = WristConstants.MMVel; // 5 rotations per second cruise
        mm.MotionMagicAcceleration = WristConstants.MMAcc;   // Take approximately 0.5 seconds to reach max vel 
        mm.MotionMagicJerk = WristConstants.MMJerk;          // Take approximately 0.2 seconds to reach max accel

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = 20;
        slot0.kI = 0;
        slot0.kD = 0.0078125;
        slot0.kV = 0.009375;
        slot0.kS = 0.01953125; // Approximately 0.25V to get the mechanism moving

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1;

        // Apply the configs for Motor 1
        cfg.MotorOutput.Inverted = WristConstants.wrist2Direction;
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            status = wristMotor1.getConfigurator().apply(cfg);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }

        // Apply the configs for Motor 2
        cfg.MotorOutput.Inverted = WristConstants.wrist1Direction;
        status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
            status = wristMotor2.getConfigurator().apply(cfg);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
        
        wristMotor1.setPosition(0);
        wristMotor2.setPosition(0);
    }

    public void periodic() {
        if(!enabled) return;
        SmartDashboard.putNumber("Wrist motor 1 position", wristMotor1.getRotorPosition().getValueAsDouble());
        SmartDashboard.putNumber("Wrist motor 2 position", wristMotor2.getRotorPosition().getValueAsDouble());


        SmartDashboard.putNumber("wrist desired position", desiredPosition);
        SmartDashboard.putNumber("wrist motor 1 closedLoopError", wristMotor1.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("wrist motor 1 closedLoopError", wristMotor1.getClosedLoopError().getValueAsDouble());
        SmartDashboard.putNumber("wrist motor 1 closedLoopReference", wristMotor1.getClosedLoopReference().getValueAsDouble());     
        SmartDashboard.putNumber("wrist motor 2 closedLoopReference", wristMotor2.getClosedLoopReference().getValueAsDouble());  
        SmartDashboard.putNumber("wrist motor 1 closedLoopOutput", wristMotor1.getClosedLoopOutput().getValueAsDouble());    
        SmartDashboard.putNumber("wrist motor 2 closedLoopOutput", wristMotor2.getClosedLoopOutput().getValueAsDouble()); 
        SmartDashboard.putNumber("wrist motor 1 statorCurrent", wristMotor1.getStatorCurrent().getValueAsDouble());    
        SmartDashboard.putNumber("wrist motor 2 statorCurrent", wristMotor2.getStatorCurrent().getValueAsDouble());  

        // SmartDashboard.putString("Vision pose", String.format("(%.2f, %.2f) %.2f",
        //                         estPose.getTranslation().getX(),
        //                         estPose.getTranslation().getY(),
        //                         estPose.getRotation().getDegrees()));
       //qqpppppppppppppppppppppppppppppp-dpd-fg= SmartDashboard.putNumber("Arbitrary Feed Forward", arbitraryFeedForward);

    }

    public double getWristPosition() {
        if(!enabled) return 0;
        return wristMotor1.getPosition().getValueAsDouble();
    }

    public void extendTrapFlap() {
        trapFlapServo.setPosition(0);
        System.out.println("extending");
    }

    public void retractTrapFlap() {
      trapFlapServo.setPosition(45);
      System.out.println("retracting");
    }

   public void slowlyDown() {
        wristMotor1.setControl(new DutyCycleOut(-.1));
        wristMotor2.setControl(new DutyCycleOut(-.1));
    }
   public void stop() {
        wristMotor1.setPosition(0);
        wristMotor2.setPosition(0);
        wristMotor1.setControl(new DutyCycleOut(0));
        wristMotor2.setControl(new DutyCycleOut(0));
        mmReq.withVelocity(WristConstants.MMVel);
        wristMotor1.setControl(mmReq.withPosition(0).withFeedForward(arbitraryFeedForward));
        wristMotor2.setControl(mmReq.withPosition(0).withFeedForward(arbitraryFeedForward));
        desiredPosition = 0.0;
    }

    private boolean isRedAlliance(){
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance != null){
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public void setWristBasedOnDistance(double distanceToSpeakerInMeters, double xDistanceToSpeaker, double yDistanceToSpeaker, double leftX, double leftY, double accelerationXPigeon, double accelerationYPigeon, double robotHeading) {
         
        double shotTime = distanceToSpeakerInMeters / (3.81 * 2 * Math.PI); //speed of shot in m/s
        
        double robotXSpeed = driveSubsystem.getXVelocity();
        double robotYSpeed = driveSubsystem.getYVelocity();
        
        double fieldRelativeRobotXSpeed = (robotXSpeed * Math.sin(robotHeading)) + (robotYSpeed * Math.cos(robotHeading));
        double fieldRelativeRobotYSpeed = (robotXSpeed * Math.cos(robotHeading)) + (robotYSpeed * Math.sin(robotHeading));

        double robotXSetSpeed = -(leftY*leftY * MaxSpeed);
        double robotYSetSpeed = -(leftX*leftX * MaxSpeed);
         
        double robotXAcceleration = accelerationXPigeon;
        double robotYAcceleration = accelerationYPigeon;
        
        double fieldRelativeRobotXAcceleration = (robotXAcceleration * Math.sin(robotHeading)) + (robotYAcceleration * Math.cos(robotHeading));
        double fieldRelativeRobotYAcceleration = (robotXAcceleration * Math.cos(robotHeading)) + (robotYAcceleration * Math.sin(robotHeading));
        //if statements determine if difference in current robot speed and set robot speed is indicative of acceleration, and in which direction
        // if (robotXSetSpeed < robotXSpeed && Math.abs(robotXSetSpeed - robotXSetSpeed) >= 0.5){
        //     robotXAcceleration = -4;
        // } if (robotXSetSpeed > robotXSpeed && Math.abs(robotXSetSpeed - robotXSetSpeed) >= 0.5) {
        //     robotXAcceleration = 4;
        // } else {
        //     robotXAcceleration = 0;
        // }

        // if (robotYSetSpeed < robotYSpeed && Math.abs(robotYSetSpeed - robotYSetSpeed) >= 0.5){
        //     robotYAcceleration = -4;
        // } if (robotYSetSpeed > robotYSpeed && Math.abs(robotYSetSpeed - robotYSetSpeed) >= 0.5) {
        //    robotYAcceleration = 4;
        // } else {
        //    robotYAcceleration = 0;
        // }
        
        //code assumes that X velocity and acceleration toward the blue alliance speaker is positive, and negative for red alliance speaker

        double m = 1.0;
        if(isRedAlliance()){
           m = -1.0;
        }
        //code assumes that Y velocity in direction of speaker is positive, and negative going away
        double m2 = 1.0;
        if(m_visionSubsystem.isBelowSpeakerLine()){
           m2 = -1.0;
        }


        double robotXDistance = xDistanceToSpeaker - ((robotXSpeed * shotTime) * m) - ((0.5 * robotXAcceleration * Math.pow(shotTime, 2)) * m);
        double robotYDistance = yDistanceToSpeaker - ((robotYSpeed * shotTime) * m2) - ((0.5 * robotYAcceleration * Math.pow(shotTime, 2)) * m2);
       //double distanceFromOrigin = Math.sqrt((robotXDistance * robotXDistance) + (robotYDistance * robotYDistance));
       double distanceFromOrigin = distanceToSpeakerInMeters;
        //old formula
        //double angle =  -1.7118 * Math.pow(distanceFromOrigin,4)+ 22.295* Math.pow(distanceFromOrigin,3) - 106.7* Math.pow(distanceFromOrigin,2) + 226.57* distanceFromOrigin -165.56;
        double angle =  (-0.0954 * Math.pow(distanceFromOrigin,4)) + (1.6964* Math.pow(distanceFromOrigin,3)) - (11.647* Math.pow(distanceFromOrigin,2)) + (38.352* distanceFromOrigin) -34.77;
        SmartDashboard.putNumber("Angle",angle);
        if (angle > 0 && angle < 23.4) {
            moveWrist(angle);
        }
        

    }


    public void jogDown() {
        wristMotor1.setControl(new DutyCycleOut(-.1));
        wristMotor2.setControl(new DutyCycleOut(-.1));
    }

        public void jogUp() {
        wristMotor1.setControl(new DutyCycleOut(.2));
        wristMotor2.setControl(new DutyCycleOut(.2));
    }

    public void stopJog() {
        wristMotor1.setControl(new DutyCycleOut(0));
        wristMotor2.setControl(new DutyCycleOut(0));
        mmReq.withVelocity(WristConstants.MMVel);
        wristMotor1.setControl(mmReq.withPosition(getWristPosition()).withFeedForward(arbitraryFeedForward));
        wristMotor2.setControl(mmReq.withPosition(getWristPosition()).withFeedForward(arbitraryFeedForward));
        desiredPosition = getWristPosition();
    }
}
