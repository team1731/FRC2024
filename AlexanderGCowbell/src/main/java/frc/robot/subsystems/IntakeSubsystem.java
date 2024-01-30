package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem  extends SubsystemBase implements ToggleableSubsystem {
    private CANSparkMax intakeMotor;
    private CANSparkMax feederMotor;
    private DigitalInput noteTrigger = new DigitalInput(0);
    
    private boolean enabled;
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public IntakeSubsystem(boolean enabled) {
        this.enabled = enabled;
        System.out.println("IntakeSubsystem: Starting up!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        if (enabled) {
            initializeIntakeMotor();
        }
    }

    private void initializeIntakeMotor() {
        System.out.println("IntakeSubsystem: Initializing arm motors!!!!!!!!!!!!!!!!!!!!!!!!!");
        if (enabled) {
            intakeMotor = new CANSparkMax(IntakeConstants.intakeCancoderId, MotorType.kBrushless);
            intakeMotor.restoreFactoryDefaults();
            intakeMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_A);
            intakeMotor.setInverted(false);
            intakeMotor.setIdleMode(IdleMode.kCoast);
            
            feederMotor = new CANSparkMax(IntakeConstants.feederCancoderId, MotorType.kBrushless);
            feederMotor.restoreFactoryDefaults();
            feederMotor.setSmartCurrentLimit(IntakeConstants.FEEDER_CURRENT_LIMIT_A);
            feederMotor.setInverted(false);
            feederMotor.setIdleMode(IdleMode.kBrake);
        }
    }

    /*
     * INTAKE MOTOR MOVEMENT
     */

    private void intake(double intakeSpeed) {
        if (enabled) {
            // intakeMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_A);
            if (!noteTrigger.get()) {
                intakeSpeed = 0;
            }
            intakeMotor.set(intakeSpeed);
            System.out.println("IntakeSubsystem: speed = " + intakeSpeed);
        }
    }

    public void intake() {
        intake(IntakeConstants.intakeSpeed);
    }    

    public void reverseIntake() {
        double intakeSpeed = -1 * IntakeConstants.intakeSpeed;
        System.out.println("IntakeSubsystem: reverse speed = " + intakeSpeed);
        intake(intakeSpeed);
    }    

    public void stopIntake() {
        if (enabled) {
            intakeMotor.set(0);
        }
    }
    
    /*
     * FEEDER MOTOR MOVEMENT
     */

    public void feeder() {
        if (enabled) {
            feederMotor.set(IntakeConstants.feederSpeed);
        }
    }

    public void reverseFeeder() {
        if (enabled) {
            feederMotor.set(-IntakeConstants.feederSpeed);
        }
    }

    public void stopFeeder() {
        if (enabled) {
            feederMotor.set(0);
        }
    }

    /* Combined Motor Movement */
    public void grabOrangeNote() {
        if (enabled) {
            if (!noteTrigger.get()) {
                intakeMotor.set(0);
                feederMotor.set(0);
            } else {
                intakeMotor.set(IntakeConstants.intakeSpeed);
                feederMotor.set(IntakeConstants.feederSpeed);
            }
        }
    }

    public void stopOrangeNoteGrab() {
        if (enabled) {
            intakeMotor.set(0);
            feederMotor.set(0);
        }
    }

    /*
    public void eject() {
        if(!enabled){
            return;
        }
       intakeMotor.setSmartCurrentLimit(IntakeConstants.EJECT_CURRENT_LIMIT);
        intakeMotor.set(-1.0);
    }
    
    public void holdIntake() {
        if(!enabled){
            return;
        }
        intakeMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_HOLD_CURRENT_LIMIT_A);
        intakeMotor.set(IntakeConstants.INTAKE_HOLD_POWER);
    }

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

    public boolean isIntakeAtStartedVelocity() {
        if(!enabled){
            return true;
        }
        return (Math.abs(intakeMotor.getEncoder().getVelocity()) > IntakeConstants.intakeStartedVelocityThreshold);
    }

    public boolean isIntakeBelowStartedVelocity() {
        if(!enabled){
            return true;
        }
        return (Math.abs(intakeMotor.getEncoder().getVelocity()) < IntakeConstants.intakeStartedVelocityThreshold);
    }

    public boolean isIntakeAtHoldingVelocity() {
        if(!enabled){
            return true;
        }
        return (Math.abs(intakeMotor.getEncoder().getVelocity()) < IntakeConstants.intakeHoldingVelocityThreshold);
    }
    */
}
