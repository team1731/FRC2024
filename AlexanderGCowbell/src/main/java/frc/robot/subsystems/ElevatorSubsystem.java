package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase implements ToggleableSubsystem {

    // motors for the Elevator
    private TalonFX elevatorMotor1;
    // private TalonFX elevatorMotor2;
    private MotionMagicVoltage mmReq1;
    // private MotionMagicVoltage mmReq2;

    private boolean enabled;
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public ElevatorSubsystem(boolean enabled) {
        this.enabled = enabled;
        initializeElevatorMotor();
    }

    /*
     * Elevator MOTOR MOVEMENT
    */

    public double getElevatorPosition() {
        if (enabled){
            return elevatorMotor1.getPosition().getValueAsDouble();
        }
        else{
            return 0;
        }
    }

    private void moveElevator(double position, double maxVelocity) {
        if (enabled){
            elevatorMotor1.setControl(mmReq1.withPosition(position).withSlot(0));
            // elevatorMotor1.setControl(mmReq1.withPosition(ElevatorConstants.elevatorExtendedPosition).withSlot(0));
            // elevatorMotor2.setControl(mmReq2.withPosition(ElevatorConstants.elevatorExtendedPosition).withSlot(0));
            // elevatorMotor1.setPosition(1);
            // elevatorMotor2.setPosition(1);
        }
    }
    
    public void elevatorExtended() {
        System.out.println("X - onTrue - elevatorExtended method called");
        moveElevator(ElevatorConstants.elevatorExtendedPosition, ElevatorConstants.MMVel);
    }

    public void elevatorHome() {
        System.out.println("X - onFalse - elevatorHome method called");
        moveElevator(ElevatorConstants.elevatorHomePosition, ElevatorConstants.MMVel);
    }

    // Initialize Motors
    private void initializeElevatorMotor() {
        if (!enabled) {
            return;
        }
        System.out.println("ElevatorSubsystem: Starting UP & Initializing Elevator motors !!!!!!");

        // Initialize Motor 1
        elevatorMotor1 = new TalonFX(ElevatorConstants.elevatorCancoderId1);  // Constants.CANBUS_NAME);
        mmReq1 = new MotionMagicVoltage(0);

        TalonFXConfiguration cfg1 = new TalonFXConfiguration();

        // Factory Default
        elevatorMotor1.getConfigurator().apply(cfg1);
    
        // Elevator Motor 1 is CCW+
        cfg1.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        /* Configure current limits */
        MotionMagicConfigs mm1 = cfg1.MotionMagic;
        mm1.MotionMagicCruiseVelocity = ElevatorConstants.MMVel; // 5 rotations per second cruise
        mm1.MotionMagicAcceleration = ElevatorConstants.MMAcc; // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel 
        mm1.MotionMagicJerk = ElevatorConstants.MMJerk;
    
        // initialze PID controller and encoder objects
        Slot0Configs slot0_1 = cfg1.Slot0;
        slot0_1.kP = ElevatorConstants.kP;
        slot0_1.kI = ElevatorConstants.kI;
        slot0_1.kD = ElevatorConstants.kD;
        slot0_1.kV = ElevatorConstants.kV;
        slot0_1.kS = ElevatorConstants.kS; // Approximately 0.25V to get the mechanism moving
    
        FeedbackConfigs fdb1 = cfg1.Feedback;
        fdb1.SensorToMechanismRatio = ElevatorConstants.StM_Ratio;
    
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = elevatorMotor1.getConfigurator().apply(cfg1);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure Elevator Motor 1. Error: " + status.toString());
        }
    }

    //     // Initialize Motor 2
    //     // elevatorMotor2 = new TalonFX(ElevatorConstants.elevatorCancoderId2, "canivore");
    //     mmReq2 = new MotionMagicVoltage(0);

    //     TalonFXConfiguration cfg2 = new TalonFXConfiguration();

    //     // Factory Default
    //     // elevatorMotor2.getConfigurator().apply(cfg2);
 
    //     // Elevator Motor 2 is CW+
    //     cfg2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    //     /* Configure current limits */
    //     MotionMagicConfigs mm2 = cfg2.MotionMagic;
    //     mm2.MotionMagicCruiseVelocity = ElevatorConstants.MMVel; // 5 rotations per second cruise
    //     mm2.MotionMagicAcceleration = ElevatorConstants.MMAcc; // Take approximately 0.5 seconds to reach max vel
    //     // Take approximately 0.2 seconds to reach max accel 
    //     mm2.MotionMagicJerk = ElevatorConstants.MMJerk;

    //     // initialze PID controller and encoder objects
    //     Slot0Configs slot0_2 = cfg2.Slot0;
    //     slot0_2.kP = ElevatorConstants.kP;
    //     slot0_2.kI = ElevatorConstants.kI;
    //     slot0_2.kD = ElevatorConstants.kD;
    //     slot0_2.kV = ElevatorConstants.kV;
    //     slot0_2.kS = ElevatorConstants.kS; // Approximately 0.25V to get the mechanism moving
        
    //     FeedbackConfigs fdb2 = cfg2.Feedback;
    //     fdb2.SensorToMechanismRatio = ElevatorConstants.StM_Ratio;
 
    //     StatusCode status2 = StatusCode.StatusCodeNotInitialized;
    //     for(int i = 0; i < 5; ++i) {
    //       status2 = elevatorMotor1.getConfigurator().apply(cfg2);
    //       if (status2.isOK()) break;
    //     }
    //     if (!status2.isOK()) {
    //       System.out.println("Could not configure Elevator Motor 1. Error: " + status2.toString());
    //     }

    //     // Ensure motor 2 follows motor 1
    //     elevatorMotor2.setControl(new Follower(elevatorMotor1.getDeviceID(), false));
    // }

    // // public void reverseElevator() {
    //     if (!enabled) {
    //         return;
    //     }
    //     double elevatorSpeed = 0.75;
    //     elevatorSpeed = -1 * ElevatorConstants.minVel;
    //     System.out.println("ElevatorSubsystem: speed = " + elevatorSpeed);
    //     elevator(elevatorSpeed);
    //  }

    // public void elevator(double elevatorSpeed) {
    //     if (!enabled) {
    //         return;
    //     }
    //     elevatorMotor.setSmartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT_A);
    //     elevatorMotor.set(elevatorSpeed);
    // }

    // public void eject() {
    //     if (!enabled) {
    //         return;
    //     }
    //     elevatorMotor.setSmartCurrentLimit(ElevatorConstants.EJECT_CURRENT_LIMIT);
    //     // elevatorMotor.set((stateMachine.getGamePiece() == GamePiece.CONE)? -1.0 : 1.0);
    //     elevatorMotor.set(-1.0);
    // }

    // public void holdElevator() {
    //     if (!enabled) {
    //         return;
    //     }
    //     elevatorMotor.setSmartCurrentLimit(ElevatorConstants.ELEVATOR_HOLD_CURRENT_LIMIT_A);
    //     // elevatorMotor.set((stateMachine.getGamePiece() == GamePiece.CONE)? ArmConstants.ELEVATOR_HOLD_POWER : -1 * ArmConstants.ELEVATOR_HOLD_POWER);
    //     elevatorMotor.set(ElevatorConstants.ELEVATOR_HOLD_POWER);
    // }

    // public void stopElevator() {
    //     if (!enabled) {
    //         return;
    //     }
    //     elevatorMotor.setSmartCurrentLimit(ElevatorConstants.ELEVATOR_CURRENT_LIMIT_A);
    //     elevatorMotor.set(0);
    // }

    // public boolean isElevatorAtStartedVelocity() {
    //     if (!enabled) {
    //         return true;
    //     }
    //     return (Math.abs(elevatorMotor.getEncoder().getVelocity()) > ElevatorConstants.elevatorStartedVelocityThreshold);
    // }

    // public boolean isElevatorBelowStartedVelocity() {
    //     if (!enabled) {
    //         return false;
    //     }
    //     return (Math.abs(elevatorMotor.getEncoder().getVelocity()) < ElevatorConstants.elevatorStartedVelocityThreshold);
    // }

    // public boolean isElevatorAtHoldingVelocity() {
    //     if (!enabled) {
    //         return true;
    //     }
    //     return (Math.abs(elevatorMotor.getEncoder().getVelocity()) < ElevatorConstants.elevatorHoldingVelocityThreshold);
    // }
}
