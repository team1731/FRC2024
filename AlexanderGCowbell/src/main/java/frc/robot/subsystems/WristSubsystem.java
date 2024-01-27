package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class WristSubsystem  extends SubsystemBase implements ToggleableSubsystem {
    private CANSparkMax wristMotor;
    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private DigitalInput killSwitch = new DigitalInput(0);
    private double last_encoder = 0;
    
    private boolean enabled;
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public WristSubsystem(boolean enabled) {
        this.enabled = enabled;
        System.out.println("WristSubsystem: Starting up!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        if (enabled) {
            initializeWristMotor();
        }
    }

    private void initializeWristMotor() {
        System.out.println("WristSubsystem: Initializing arm motors!!!!!!!!!!!!!!!!!!!!!!!!!");
        if(!enabled){
            return;
        }
        wristMotor = new CANSparkMax(WristConstants.wristCancoderId, MotorType.kBrushless);
        wristMotor.restoreFactoryDefaults();
        wristMotor.setSmartCurrentLimit(WristConstants.WRIST_CURRENT_LIMIT_A);
        wristMotor.setInverted(false);
        wristMotor.setIdleMode(IdleMode.kBrake);

        // initialze PID controller and encoder objects
        m_pidController = wristMotor.getPIDController();
        m_encoder = wristMotor.getEncoder();

        // set PID coefficients
        m_pidController.setP(WristConstants.kP);
        m_pidController.setI(WristConstants.kI);
        m_pidController.setD(WristConstants.kD);
        m_pidController.setIZone(WristConstants.kIz);
        m_pidController.setFF(WristConstants.kFF);
        m_pidController.setOutputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);   
        
        /**
        * Smart Motion coefficients are set on a SparkPIDController object
        * 
        * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
        * the pid controller in Smart Motion mode
        * - setSmartMotionMinOutputVelocity() will put a lower bound in
        * RPM of the pid controller in Smart Motion mode
        * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
        * of the pid controller in Smart Motion mode
        * - setSmartMotionAllowedClosedLoopError() will set the max allowed
        * error for the pid controller in Smart Motion mode
        */

       m_pidController.setSmartMotionMaxVelocity(WristConstants.maxVel, WristConstants.smartMotionSlot);
       m_pidController.setSmartMotionMinOutputVelocity(WristConstants.minVel, WristConstants.smartMotionSlot);
       m_pidController.setSmartMotionMaxAccel(WristConstants.maxAcc, WristConstants.smartMotionSlot);
       m_pidController.setSmartMotionAllowedClosedLoopError(WristConstants.allowedErr, WristConstants.smartMotionSlot);

        m_pidController.setOutputRange(WristConstants.minVel, WristConstants.maxVel);
    }

    /*
     * WRIST MOTOR MOVEMENT
     */

    public void wristUp() {
        last_encoder = m_encoder.getPosition();
        m_pidController.setReference(last_encoder + 1000, CANSparkMax.ControlType.kSmartMotion);
    }

    public void wristDown() {
        m_pidController.setReference(last_encoder, CANSparkMax.ControlType.kSmartMotion);
    }

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
        if (!killSwitch.get()) {
            wristSpeed = 0;
        }
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

}
