package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.util.log.LogWriter;

public class WristSubsystem  extends SubsystemBase implements ToggleableSubsystem {
    private CANSparkMax wristMotor;
    private SparkPIDController wristPIDController;
    private RelativeEncoder wristEncoder;

    private boolean enabled;
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public WristSubsystem(boolean enabled) {
        this.enabled = enabled;
        if (enabled) {
            System.out.println("WristSubsystem: Starting up!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            initializeWristMotor();
            setWristEncoder(0); // initialize to 0
        }
    }

    private void initializeWristMotor() {
        if(!enabled){
            return;
        }
        System.out.println("WristSubsystem: Initializing arm motors!!!!!!!!!!!!!!!!!!!!!!!!!");
        wristMotor = new CANSparkMax(WristConstants.wristCancoderId, MotorType.kBrushless);
        wristMotor.restoreFactoryDefaults();
        wristMotor.setSmartCurrentLimit(WristConstants.WRIST_CURRENT_LIMIT_A);
        // wristMotor.setInverted(false);
        wristMotor.setIdleMode(IdleMode.kBrake);

        // initialze PID controller and encoder objects
        wristPIDController = wristMotor.getPIDController();
        wristEncoder = wristMotor.getEncoder();

        // set PID coefficients
        wristPIDController.setP(WristConstants.kP);
        wristPIDController.setI(WristConstants.kI);
        wristPIDController.setD(WristConstants.kD);
        wristPIDController.setIZone(WristConstants.kIz);
        wristPIDController.setFF(WristConstants.kFF);
        wristPIDController.setOutputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);   
        //wristPIDController.setFeedbackDevice();
        
        wristPIDController.setSmartMotionMaxVelocity(WristConstants.maxVel, WristConstants.smartMotionSlot);
        wristPIDController.setSmartMotionMinOutputVelocity(WristConstants.minVel, WristConstants.smartMotionSlot);
        wristPIDController.setSmartMotionMaxAccel(WristConstants.maxAcc, WristConstants.smartMotionSlot);
        wristPIDController.setSmartMotionAllowedClosedLoopError(WristConstants.allowedErr, WristConstants.smartMotionSlot);

        // wristPIDController.setReference(0, CANSparkMax.ControlType.kSmartMotion);
    }

    /*
     * WRIST MOTOR MOVEMENT
     */
    public void setWristEncoder(double position) {
        if (enabled){
            wristEncoder.setPosition(position);
        }
    }

    public double getWristPosition() {
        if (enabled){
            return wristEncoder.getPosition();
        }
        else{
            return 0;
        }
    }

    private void moveWrist(double position, double maxVelocity) {
        if (enabled){
            System.out.println("WristSubsystem: moveWrist = " + position);
            wristPIDController.setSmartMotionMaxVelocity(maxVelocity, WristConstants.smartMotionSlot);
            wristPIDController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
        }
    }
    
    public void printPosition() {
        System.out.println("WristSubsystem: absPos = " + wristEncoder.getPosition());
    }

    public void wristExtended() {
        System.out.println("WristSubsystem: B-absPos = " + wristEncoder.getPosition());
        moveWrist(WristConstants.wristExtendedPosition, WristConstants.maxVel);
        System.out.println("WristSubsystem: A-absPos = " + wristEncoder.getPosition());
    }

    public void wristHome() {
        System.out.println("WristSubsystem: B-absPos = " + wristEncoder.getPosition());
        moveWrist(WristConstants.wristHomePosition, WristConstants.maxVel);
        System.out.println("WristSubsystem: A-absPos = " + wristEncoder.getPosition());
    }

    /*
    public void multiplyInput(double value) {
        // multiplied value
        double current = value * WristConstants.wristSpeed;
        // minimum speed variable
        final double minimum = .1;
        if (current < minimum && current > -minimum) {
            current = 0;
        }
        wrist(current);
     }

    public void wrist() {
        double wristSpeed = WristConstants.wristSpeed;
        System.out.println("WristSubsystem: speed = " + wristSpeed);
        wrist(wristSpeed);
    }

    public void reverseWrist() {
        double wristSpeed = -1 * WristConstants.wristSpeed;
        System.out.println("WristSubsystem: speed = " + wristSpeed);
        wrist(wristSpeed);
    }

    public void wrist(double wristSpeed) {
        if(!enabled){
            return;
        }
        wristMotor.setSmartCurrentLimit(WristConstants.WRIST_CURRENT_LIMIT_A);
        wristMotor.set(wristSpeed);
    }

    public void feederWrist() {
         if(!enabled){
            return;
        }
       // wristMotor.setSmartCurrentLimit(WristConstants.WRIST_CURRENT_LIMIT_A);
        wristMotor.set(0);
    }

    public void eject() {
        if(!enabled){
            return;
        }
       wristMotor.setSmartCurrentLimit(WristConstants.EJECT_CURRENT_LIMIT);
        wristMotor.set(-1.0);
    }

    public void holdWrist() {
        if(!enabled){
            return;
        }
        wristMotor.setSmartCurrentLimit(WristConstants.WRIST_HOLD_CURRENT_LIMIT_A);
        wristMotor.set(WristConstants.WRIST_HOLD_POWER);
    }

    public void stopWrist() {
        if(!enabled){
            return;
        }
        wristMotor.setSmartCurrentLimit(WristConstants.WRIST_CURRENT_LIMIT_A);
        wristMotor.set(0);
    }

    public boolean isWristAtStartedVelocity() {
        if(!enabled){
            return true;
        }
        return (Math.abs(wristMotor.getEncoder().getVelocity()) > WristConstants.wristStartedVelocityThreshold);
    }

    public boolean isWristBelowStartedVelocity() {
        if(!enabled){
            return true;
        }
        return (Math.abs(wristMotor.getEncoder().getVelocity()) < WristConstants.wristStartedVelocityThreshold);
    }

    public boolean isWristAtHoldingVelocity() {
        if(!enabled){
            return true;
        }
        return (Math.abs(wristMotor.getEncoder().getVelocity()) < WristConstants.wristHoldingVelocityThreshold);
    }
    */
}
