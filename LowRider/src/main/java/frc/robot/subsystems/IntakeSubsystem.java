package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
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
    private boolean noteStopEnabled = true;
    private SparkLimitSwitch m_forwardLimit;
    private SparkLimitSwitch m_reverseLimit;
    private boolean isIntaking = false;
   
    
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
            m_reverseLimit = feederMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
            disableReverseLimitSwitch();


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
        intake(IntakeConstants.intakeSpeed);
        feed();

    }    

    public void reverseIntake() {
        double intakeSpeed = -1 * IntakeConstants.intakeSpeed;
        System.out.println("IntakeSubsystem: reverse speed = " + intakeSpeed);
        intake(intakeSpeed);
        reverseFeed();
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

    public void feed() {
        if (enabled) {
            feederMotor.set(IntakeConstants.feederSpeed);
        }
    }

    public void reverseFeed() {
        if (enabled) {
            feederMotor.set(-IntakeConstants.feederSpeed);
        }
    }

    public void trapFeed() {
        if (enabled) {
            feederMotor.set(-IntakeConstants.trapFeedSpeed);
        }
    }

    public void stopFeed() {
        if (enabled) {
            feederMotor.set(0);
        }
    }

   
    
    public void periodic() {

        if ((feederMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed()) && isIntaking) {
            reverseIntake();
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

    public boolean noteIsPresent() {
        // TODO Auto-generated method stub
       return (feederMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed());
    }


}
