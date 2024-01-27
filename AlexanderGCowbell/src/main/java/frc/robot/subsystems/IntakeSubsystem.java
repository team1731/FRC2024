package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.state.arm.ArmStateMachine.MovementType;

public class IntakeSubsystem  extends SubsystemBase implements ToggleableSubsystem {
    private CANSparkMax intakeMotor;
    private CANSparkMax feederMotor;
    private DigitalInput killSwitch = new DigitalInput(0);
    
    private boolean enabled;
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public IntakeSubsystem(boolean enabled) {
        System.out.println("IntakeSubsystem: Starting up!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        if (enabled) {
            initializeIntakeMotor();
        }
    }

    private void initializeIntakeMotor() {
        System.out.println("IntakeSubsystem: Initializing arm motors!!!!!!!!!!!!!!!!!!!!!!!!!");
        intakeMotor = new CANSparkMax(IntakeConstants.intakeCancoderId, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_A);
        intakeMotor.setInverted(false);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        
        feederMotor = new CANSparkMax(IntakeConstants.feederCancoderId, MotorType.kBrushless);
        feederMotor.restoreFactoryDefaults();
        feederMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_A);
        feederMotor.setInverted(false);
        feederMotor.setIdleMode(IdleMode.kBrake);
    }

    /*
     * INTAKE MOTOR MOVEMENT
     */

     public void multiplyInput(double value) {
        // multiplied value
        double current = value * ArmConstants.cubeIntakeSpeed;
        // minimum speed variable
        final double minimum = .1;
        if (current < minimum && current > -minimum) {
            current = 0;
        }
        intake(current);
     }

    public void intake() {
        double intakeSpeed = IntakeConstants.intakeSpeed;
        System.out.println("IntakeSubsystem: speed = " + intakeSpeed);
        intake(intakeSpeed);
    }

    public void reverseIntake() {
        double intakeSpeed = -1 * IntakeConstants.intakeSpeed;
        System.out.println("IntakeSubsystem: speed = " + intakeSpeed);
        intake(intakeSpeed);
    }

    public void intake(double intakeSpeed) {
        intakeMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_A);
        if (!killSwitch.get()) {
            intakeSpeed = 0;
        }
        intakeMotor.set(intakeSpeed);
        feederMotor.set(intakeSpeed);
    }

    public void feederIntake() {
        // intakeMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_A);
        intakeMotor.set(0);
        feederMotor.set(IntakeConstants.feederSpeed);
    }

    public void stopFeederIntake() {
        feederMotor.set(0);
    }

    public void reverseFeederIntake() {
        feederMotor.set(-IntakeConstants.feederSpeed);
    }

    public void eject() {
        intakeMotor.setSmartCurrentLimit(IntakeConstants.EJECT_CURRENT_LIMIT);
        intakeMotor.set(-1.0);
    }

    public void holdIntake() {
        intakeMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_HOLD_CURRENT_LIMIT_A);
        intakeMotor.set(IntakeConstants.INTAKE_HOLD_POWER);
    }

    public void stopIntake() {
        intakeMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_A);
        intakeMotor.set(0);
    }

    public boolean isIntakeAtStartedVelocity() {
        return (Math.abs(intakeMotor.getEncoder().getVelocity()) > IntakeConstants.intakeStartedVelocityThreshold);
    }

    public boolean isIntakeBelowStartedVelocity() {
        return (Math.abs(intakeMotor.getEncoder().getVelocity()) < IntakeConstants.intakeStartedVelocityThreshold);
    }

    public boolean isIntakeAtHoldingVelocity() {
        return (Math.abs(intakeMotor.getEncoder().getVelocity()) < IntakeConstants.intakeHoldingVelocityThreshold);
    }

}
