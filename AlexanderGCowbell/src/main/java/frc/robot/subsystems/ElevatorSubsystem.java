package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem  extends SubsystemBase implements ToggleableSubsystem {
    private CANSparkMax elevatorMotor;

    private SparkPIDController elevatorPIDController;
    private AbsoluteEncoder elevatorEncoder;

    private boolean enabled;
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public ElevatorSubsystem(boolean enabled) {
        this.enabled = enabled;
        System.out.println("ElevatorSubsystem: Starting up!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        if (enabled) {
            initializeElevatorMotor();
        }
    }

    private void initializeElevatorMotor() {
        if (!enabled) {
            return;
        }
        System.out.println("ElevatorSubsystem: Initializing arm motors!!!!!!!!!!!!!!!!!!!!!!!!!");
        elevatorMotor = new CANSparkMax(ElevatorConstants.elevatorCancoderId, MotorType.kBrushless);
        elevatorMotor.restoreFactoryDefaults();
        elevatorMotor.setSmartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT_A);
        elevatorMotor.setInverted(false);
        elevatorMotor.setIdleMode(IdleMode.kBrake);

        // initialze PID controller and encoder objects
        elevatorPIDController = elevatorMotor.getPIDController();
        elevatorEncoder = elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle);

        // set PID coefficients
        elevatorPIDController.setP(ElevatorConstants.kP);
        elevatorPIDController.setI(ElevatorConstants.kI);
        elevatorPIDController.setD(ElevatorConstants.kD);
        elevatorPIDController.setIZone(ElevatorConstants.kIz);
        elevatorPIDController.setFF(ElevatorConstants.kFF);
        elevatorPIDController.setOutputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);   
        elevatorPIDController.setFeedbackDevice(elevatorEncoder);
        
        elevatorPIDController.setSmartMotionMaxVelocity(ElevatorConstants.maxVel, ElevatorConstants.smartMotionSlot);
        elevatorPIDController.setSmartMotionMinOutputVelocity(ElevatorConstants.minVel, ElevatorConstants.smartMotionSlot);
        elevatorPIDController.setSmartMotionMaxAccel(ElevatorConstants.maxAcc, ElevatorConstants.smartMotionSlot);
        elevatorPIDController.setSmartMotionAllowedClosedLoopError(ElevatorConstants.allowedErr, ElevatorConstants.smartMotionSlot);

        elevatorPIDController.setOutputRange(ElevatorConstants.minVel, ElevatorConstants.maxVel);
    }

    /*
     * Elevator MOTOR MOVEMENT
     */

    public double getElevatorPosition() {
        if (enabled){
            return elevatorEncoder.getPosition();
        }
        else{
            return 0;
        }
    }

    private void moveElevator(double position, double maxVelocity) {
        if (enabled){
            elevatorPIDController.setSmartMotionMaxVelocity(maxVelocity, ElevatorConstants.smartMotionSlot);
            elevatorPIDController.setReference(position, CANSparkMax.ControlType.kSmartMotion);
        }
    }
    
    public void elevatorExtended() {
        moveElevator(ElevatorConstants.elevatorExtendedPosition, ElevatorConstants.maxVel);
    }

    public void elevatorHome() {
        moveElevator(ElevatorConstants.elevatorHomePosition, ElevatorConstants.maxVel);
    }

    public void reverseElevator() {
        if (!enabled) {
            return;
        }
        double elevatorSpeed = 0.75;
        elevatorSpeed = -1 * ElevatorConstants.minVel;
        System.out.println("ElevatorSubsystem: speed = " + elevatorSpeed);
        elevator(elevatorSpeed);
     }

    public void elevator(double elevatorSpeed) {
        if (!enabled) {
            return;
        }
        elevatorMotor.setSmartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT_A);
        elevatorMotor.set(elevatorSpeed);
    }

    public void eject() {
        if (!enabled) {
            return;
        }
        elevatorMotor.setSmartCurrentLimit(ElevatorConstants.EJECT_CURRENT_LIMIT);
        // elevatorMotor.set((stateMachine.getGamePiece() == GamePiece.CONE)? -1.0 : 1.0);
        elevatorMotor.set(-1.0);
    }

    public void holdElevator() {
        if (!enabled) {
            return;
        }
        elevatorMotor.setSmartCurrentLimit(ElevatorConstants.ELEVATOR_HOLD_CURRENT_LIMIT_A);
        // elevatorMotor.set((stateMachine.getGamePiece() == GamePiece.CONE)? ArmConstants.ELEVATOR_HOLD_POWER : -1 * ArmConstants.ELEVATOR_HOLD_POWER);
        elevatorMotor.set(ElevatorConstants.ELEVATOR_HOLD_POWER);
    }

    public void stopElevator() {
        if (!enabled) {
            return;
        }
        elevatorMotor.setSmartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT_A);
        elevatorMotor.set(0);
    }

    public boolean isElevatorAtStartedVelocity() {
        if (!enabled) {
            return true;
        }
        return (Math.abs(elevatorMotor.getEncoder().getVelocity()) > ElevatorConstants.elevatorStartedVelocityThreshold);
    }

    public boolean isElevatorBelowStartedVelocity() {
        if (!enabled) {
            return false;
        }
        return (Math.abs(elevatorMotor.getEncoder().getVelocity()) < ElevatorConstants.elevatorStartedVelocityThreshold);
    }

    public boolean isElevatorAtHoldingVelocity() {
        if (!enabled) {
            return true;
        }
        return (Math.abs(elevatorMotor.getEncoder().getVelocity()) < ElevatorConstants.elevatorHoldingVelocityThreshold);
    }

}
