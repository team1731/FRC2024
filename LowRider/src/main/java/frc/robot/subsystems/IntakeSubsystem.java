package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
// import frc.robot.Robot;


public class IntakeSubsystem  extends SubsystemBase implements ToggleableSubsystem {
    private CANSparkMax intakeMotor;
    private CANSparkMax feederMotor;
    private boolean noteStopEnabled = true;
    private SparkLimitSwitch m_forwardLimit;
    private SparkLimitSwitch m_reverseLimit;
    private boolean isIntaking = false;
    private boolean noteIsRetrieved = false;
   
    
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
            feederMotor.setInverted(true);
            feederMotor.setIdleMode(IdleMode.kBrake);
            m_forwardLimit = feederMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
            m_reverseLimit = feederMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
            enableReverseLimitSwitch();


        }
    }

    /*
     * INTAKE MOTOR MOVEMENT
     */

    private void intake(double intakeSpeed) {
        if (enabled) {

            intakeMotor.set(intakeSpeed);
            feederMotor.set(intakeSpeed);
            isIntaking = true;
            //System.out.println("IntakeSubsystem: speed = " + intakeSpeed);
        }
    }

    public void intake() {
        noteIsRetrieved = false;
        intake(IntakeConstants.intakeSpeed);
        feed(1.0);

    }    

    public void reverseIntake() {
        double intakeSpeed = -1 * IntakeConstants.intakeSpeed;
        System.out.println("IntakeSubsystem: reverse speed = " + intakeSpeed);
        intake(intakeSpeed);
        feed(-1);
    }    
    public void reverseIntakeSlow() {
        double intakeSpeed = -1 * IntakeConstants.intakeSpeed;
        System.out.println("IntakeSubsystem: reverse speed = " + intakeSpeed);
        intake(intakeSpeed);
        feed(-0.2);
    }    
 

    public void stopIntake() {
        if (enabled) {
            intakeMotor.set(0);
            feederMotor.set(0);
            isIntaking = false;
        }
    }
    
    /*
     * FEEDER MOTOR MOVEMENT
     */
    public void shootAmp(double speed) {
        if (enabled) {
            isIntaking = false;
            disableLimitSwitch();
            
            feederMotor.set(speed);
        }
    }
    public void feed(double speed) {
        if (enabled) {
            feederMotor.set(speed);
        }
    }

    

    public void trapFeed() {
        if (enabled) {
            System.out.println("trapping!!!!!!!!!!!!!!!!!!!!!");
            isIntaking = false;
            disableReverseLimitSwitch();
            feederMotor.set(-IntakeConstants.trapFeedSpeed);
        }
    }

    public void stopFeed() {
        if (enabled) {
            feederMotor.set(0);
        }
    }

   
    
    public void periodic() {

        if (isIntaking &&(!feederMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed())) {
            
            reverseIntakeSlow();
            noteIsRetrieved = true;
            
        }     
    }

    public void disableLimitSwitch() {
        m_forwardLimit.enableLimitSwitch(false);
    }

    public void disableReverseLimitSwitch() {
        m_reverseLimit.enableLimitSwitch(false);
    }

    public void enableLimitSwitch() {
        m_forwardLimit.enableLimitSwitch(true);
    }

    public void enableReverseLimitSwitch() {
        m_reverseLimit.enableLimitSwitch(true);
    }

   // public boolean noteIsNotPresent() {

    //   return (!feederMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed());
   // }

    public boolean noteRetrieved() {
    
      return noteIsRetrieved;
    }

    public void fireNote() {

        disableLimitSwitch();		
		feed(1.0);
        isIntaking = false;
    }

}
