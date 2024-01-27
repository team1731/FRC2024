package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.state.arm.ArmStateMachine.MovementType;

public class ElevatorSubsystem  extends SubsystemBase implements ToggleableSubsystem {
    private CANSparkMax elevatorMotor;

    private SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

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

        m_pidController = elevatorMotor.getPIDController();
        m_encoder = elevatorMotor.getEncoder();

        kP = 5e-5; 
        kI = 1e-6;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000156; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    }

    /*
     * elevator MOTOR MOVEMENT
     */

    // public void elevator() {
    //     double elevatorSpeed = 0.75;
    //     if(stateMachine.getMovementType() == MovementType.PICKUP_DOWNED_CONE) {
    //         elevatorSpeed = ArmConstants.downedConeelevatorSpeed;
    //         System.out.println("ElevatorSubsystem: intaking DOWNED CONE, speed = " + elevatorSpeed);
    //     } else if(stateMachine.getGamePiece() == GamePiece.CONE) {
    //         elevatorSpeed =  ArmConstants.coneElevatorSpeed;
    //         System.out.println("ElevatorSubsystem: intaking CONE, speed = " + elevatorSpeed);
    //     } else {
    //         elevatorSpeed = ArmConstants.cubeElevatorSpeed;
    //         System.out.println("ElevatorSubsystem: intaking CUBE, speed = " + elevatorSpeed);
    //     }
    //     elevatorSpeed = ArmConstants.cubeElevatorSpeed;
    //     System.out.println("elevatorSubsystem: speed = " + elevatorSpeed);
    //     elevator(elevatorSpeed);
    // }

    public void reverseElevator() {
        if (!enabled) {
            return;
        }
        double elevatorSpeed = 0.75;
        elevatorSpeed = -1 * ElevatorConstants.cubeElevatorSpeed;
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

    public void goToPosition(int position) {
        elevatorMotor.set(ControlMode.MotionMagic, ArmConstants.distalHomePosition);// use motion magice here <--
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
