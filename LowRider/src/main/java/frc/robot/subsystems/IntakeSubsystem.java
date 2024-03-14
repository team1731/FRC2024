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

    private SparkLimitSwitch m_forwardLimit;
    private SparkLimitSwitch m_reverseLimit;
    private boolean isIntaking = false;
    private boolean noteIsRetrieved = false;
    private boolean isJigglingFeedingUp = false;
    private double goingUpTimer = 0;
    private double doneJigglingTimer = 0;
    private boolean finishedGoingUp = false;
    private boolean backingUpComplete = false;
    private boolean sequenceComplete = false;
    private boolean isShooterAsIntake = false;
    private LEDStringSubsystem ledSubsystem;

    private boolean enabled;
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public IntakeSubsystem(boolean enabled, LEDStringSubsystem ledSubsystem) {
        this.enabled = enabled;
        this.ledSubsystem = ledSubsystem;
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
            enableLimitSwitch();
        }
    }

    public boolean forwardLimitReached(){
        return m_forwardLimit.isPressed();
    }


  public void intakeState(double intakeSpeed) {
        if (enabled) {
         //   enableLimitSwitch();
            intakeMotor.set(intakeSpeed);
           
            //System.out.println("IntakeSubsystem: speed = " + intakeSpeed);
        }
    }

    public void feedState(double feedSpeed) {
        if (enabled) {
         //   enableLimitSwitch();
            feederMotor.set(feedSpeed);
           
            //System.out.println("IntakeSubsystem: speed = " + intakeSpeed);
        }
    }

/* 
    public void intake(double intakeSpeed) {
        if (enabled) {
         //   enableLimitSwitch();
            intakeMotor.set(intakeSpeed);
            feederMotor.set(intakeSpeed);
            isIntaking = true;
             isJigglingFeedingUp = false;
            noteIsRetrieved = false;
            //System.out.println("IntakeSubsystem: speed = " + intakeSpeed);
        }
    }

    // used for unjam
    public void reverseIntake() {
        disableReverseLimitSwitch();
        double intakeSpeed = -1 * IntakeConstants.intakeSpeed;
        System.out.println("IntakeSubsystem: reverse speed = " + intakeSpeed);
        intake(intakeSpeed);
        feed(-1);

    } 
   public void stopReverseIntake() {
    enableReverseLimitSwitch();
    stopIntake();
   } 
       
   // used only to back up note
    public void reverseIntakeSlow() {
        double intakeSpeed = -.25 * IntakeConstants.intakeSpeed;
        System.out.println("IntakeSubsystem: reverse speed = " + intakeSpeed);
        intakeMotor.set(intakeSpeed);
        feed(-0.2);
    }    
 
    public void stopIntake() {
        if (enabled) {
            intakeMotor.set(0);
            feederMotor.set(0);
            isIntaking = false;
            noteIsRetrieved = false;
            isShooterAsIntake = false;
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


    public void stoptrapFeed() {
        stopFeed();
        enableReverseLimitSwitch();
        
    }

    public void stopFeed() {
        
        if (enabled) {
            feederMotor.set(0);
        }
    }



    */
    
    public void periodic() {
/* 
 
        if (((isIntaking)&&(!isJigglingFeedingUp)) &&(feederMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed())) {
            noteIsRetrieved = true;
            reverseIntakeSlow();
            isIntaking = false;
            System.out.println("note is here and reversing intake");
            
        } else if ((isJigglingFeedingUp) && goingUpTimer != 0 && Timer.getFPGATimestamp() - goingUpTimer > .1) {

             System.out.println("Backing Up");
             enableLimitSwitch();
             reverseIntakeSlow();
             doneJigglingTimer = Timer.getFPGATimestamp();
             isJigglingFeedingUp = false;
             backingUpComplete = true;  // just need to wait for note to come back down which is done below
          
        }

        if (doneJigglingTimer != 0 && backingUpComplete  && (Timer.getFPGATimestamp()- doneJigglingTimer) > 0.4) {
            sequenceComplete = true;
            doneJigglingTimer = 0;

        }
        if (isShooterAsIntake==true && noteIsPresentShooterIntake()){
            enableReverseLimitSwitch();
            isShooterAsIntake = false;
        }
        */
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
    
/* 
    public boolean noteIsPresent() {

       return getNoteSwitch();
    }
    public boolean noteIsPresentShooterIntake() {

       return (getNoteSwitch()) || 
       (!feederMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed());
    }
    public boolean noteIsPresentShooterIntake() {

       return (feederMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed()) || 
       (!feederMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed());
    }

    public boolean noteRetrieved() {
    
      return noteIsRetrieved;
    }

    public void fireNote() {

        disableLimitSwitch();	
        noteIsRetrieved = false;	
		feed(1.0);
        isIntaking = false;
    }


    public void stopFireNote() {
        enableLimitSwitch();
        stopFeed();
    }
    
     public void stopFireNoteAuto() {
        enableLimitSwitch();
    }

    public void shootAmp(double speed) {
        if (enabled) {
            isIntaking = false;
            disableLimitSwitch();
            feederMotor.set(speed);
        }
    }

    public void shootAmpStop() {
        enableLimitSwitch();
        stopFeed();
    }

    public void shooterAsIntakeFeeder() {
        disableReverseLimitSwitch();
        feed(-0.2);
        isShooterAsIntake = true;
    }

    public void feedUpJiggle() {
       System.out.println("Starting Jiggle Feed Up");
       disableLimitSwitch();
       feed(0.5);
       isJigglingFeedingUp = true;
       goingUpTimer = Timer.getFPGATimestamp();
    }

    public void stopJiggle() {
    System.out.println("Stopping Jiggle");
    stopIntake();
    sequenceComplete = false;
     enableLimitSwitch();
    }
 
    public boolean doneJiggling() {
       // System.out.println("entering done Jiggling Sequence Complete " + sequenceComplete);
        return sequenceComplete;
       
    }

    public void initializeJiggle() {
      sequenceComplete = false;
    }

    public boolean hasNote() {
        return noteIsRetrieved;
    }

*/

    public boolean hasNote() {

       return (feederMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed()) || 
       (!feederMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed());
    }

    public boolean noteSettled() {
        return feederMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed).isPressed();
    }

}
