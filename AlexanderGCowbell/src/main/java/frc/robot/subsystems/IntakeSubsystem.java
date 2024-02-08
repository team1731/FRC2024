package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
// import frc.robot.Robot;


public class IntakeSubsystem  extends SubsystemBase implements ToggleableSubsystem {
    private CANSparkMax intakeMotor;
    private CANSparkMax feederMotor;
    private DigitalInput noteSwitch = new DigitalInput(0);
    // Creates a Debouncer in "both" mode.
    Debouncer m_debouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);
    private boolean currNoteSwitch;
    private boolean prevNoteSwitch;
    
    private boolean enabled;
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public IntakeSubsystem(boolean enabled) {
        this.enabled = enabled;
        initializeIntakeMotor();
    }

    private void initializeIntakeMotor() {
        if (enabled) {
            System.out.println("IntakeSubsystem: Starting up & Initializine Intake motors !!!!!!!!!!!!!!");

            intakeMotor = new CANSparkMax(IntakeConstants.intakeCancoderId, MotorType.kBrushless);
            intakeMotor.restoreFactoryDefaults();
            intakeMotor.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT_A);
            intakeMotor.setInverted(true);
            intakeMotor.setIdleMode(IdleMode.kCoast);
            
            feederMotor = new CANSparkMax(IntakeConstants.feederCancoderId, MotorType.kBrushless);
            feederMotor.restoreFactoryDefaults();
            feederMotor.setSmartCurrentLimit(IntakeConstants.FEEDER_CURRENT_LIMIT_A);
            feederMotor.setInverted(false);
            feederMotor.setIdleMode(IdleMode.kBrake);

            currNoteSwitch = debouncedSwitch();
            prevNoteSwitch = currNoteSwitch; // force change first time
        }
    }

    /*
     * INTAKE MOTOR MOVEMENT
     */

    private void intake(double intakeSpeed) {
        if (enabled) {
            if (!debouncedSwitch()) {
                intakeSpeed = 0;
            }
            intakeMotor.set(intakeSpeed);
            feederMotor.set(intakeSpeed);
            System.out.println("IntakeSubsystem: speed = " + intakeSpeed);
            System.out.println("DIO STATE:" + noteSwitch);
        }
    }

    public void intake() {
        intake(IntakeConstants.intakeSpeed);
        feeder();
        System.out.println("DIO STATE:" + debouncedSwitch());
    }    

    public void reverseIntake() {
        double intakeSpeed = -1 * IntakeConstants.intakeSpeed;
        System.out.println("IntakeSubsystem: reverse speed = " + intakeSpeed);
        intake(intakeSpeed);
        reverseFeeder();
    }    

    public void stopIntake() {
        if (enabled) {
            intakeMotor.set(0);
            feederMotor.set(0);
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
            currNoteSwitch = debouncedSwitch();
            prevNoteSwitch = !currNoteSwitch;
            feederMotor.set(0);
        }
    }

    /* Combined Motor Movement */
    public void grabOrangeNote() {
        if (enabled) {
            // System.out.println("IntakeSubsystem grabOrangeNote(prev:" + prevNoteSwitch + "): " + currNoteSwitch + " sw: " + debouncedSwitch());
            System.out.println("DIO STATE:" + debouncedSwitch());
            if (hasNoteTriggerChanged()) {
                currNoteSwitch = debouncedSwitch();
                if (!currNoteSwitch) {
                    intakeMotor.set(0);
                    feederMotor.set(0);
                } else {
                    intakeMotor.set(IntakeConstants.intakeSpeed);
                    feederMotor.set(IntakeConstants.feederSpeed);
                }
            }
        }
    }

    public void stopOrangeNoteGrab() {
        if (enabled) {
            intakeMotor.set(0);
            feederMotor.set(0);
            currNoteSwitch = debouncedSwitch();
            prevNoteSwitch = !currNoteSwitch;
        }
    }

    // used so that we don't continuously set/clear motors
    private boolean hasNoteTriggerChanged() {
        boolean changed = false;
        currNoteSwitch = debouncedSwitch();
        //if (Robot.doSD()) {
        //     System.out.println("IntakeSubsystem Triger(prev:" + prevNoteSwitch + "): " + currNoteSwitch);
        //}
        if (prevNoteSwitch != currNoteSwitch) {
            prevNoteSwitch = currNoteSwitch;
            changed = true;
        }
        return changed;
    }

    private boolean debouncedSwitch() {
        return m_debouncer.calculate(noteSwitch.get());
    }
    
    public void periodic() {
        if (hasNoteTriggerChanged()) {
                currNoteSwitch = debouncedSwitch();
                if (!currNoteSwitch) {
                    intakeMotor.set(0);
                    feederMotor.set(0);
                } 
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
