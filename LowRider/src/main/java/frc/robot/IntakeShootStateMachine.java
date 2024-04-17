package frc.robot;

import java.lang.reflect.Method;
import java.util.HashMap;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OpConstants.LedOption;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDStringSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

enum ISState {
    ALL_STOP,
    INTAKING,
    JIGGLING_UP,
    JIGGLING_DOWN,
    SHOOTING_AT_SPEAKER,
    SHOOTING_AT_AMP,
    SHOOTING_AT_TRAP,
    SPIN_UP_SHOOTER,
    READY_TO_SHOOT,
    SPIN_DOWN_SHOOTER,
    EJECTING,
    INTAKE_SHOOTER_WAIT,
    INTAKE_SHOOTER_HAS_NOTE,
    INTAKING_NO_JIGGLE,
    SPIN_SHOOTER_TO_LOB,
    LOBBING,
    READY_TO_LOB
}
public class IntakeShootStateMachine extends SubsystemBase {
	private final IntakeSubsystem m_intakeSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final LEDStringSubsystem m_ledSubsystem;
    private final VisionSubsystem m_visionSubsystem;
    private ISState currentState;
    private ISInput currentInput;
    private HashMap<String, Method> methods;
    private int periodicRunCounter;
    private double jiggleUpTimerStarted;
    private double JIGGLE_UP_TIMER_SECONDS = 0.1;
    private double jiggleDownTimerStarted;
    private double JIGGLE_DOWN_TIMER_SECONDS = 1.0;
    private boolean robotStarted = false;
    private boolean haveNote;
    private double LOBSPEED = 50;
    private CommandXboxController xboxController = null;

    public IntakeShootStateMachine(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, LEDStringSubsystem ledSubsystem, VisionSubsystem visionSubsystem){
        m_intakeSubsystem = intakeSubsystem;
		m_shooterSubsystem = shooterSubsystem;
        m_ledSubsystem = ledSubsystem;
        m_visionSubsystem = visionSubsystem;

        methods = new HashMap<String, Method>();
        for(Object[] transition : STATE_TRANSITION_TABLE){
            String name = (String) transition[2];
            try {
                Method method = IntakeShootStateMachine.class.getMethod(name);
                if(method != null){
                    methods.put(name, method);
                }
            } catch (NoSuchMethodException e) {
                System.out.println(" ** " + Timer.getFPGATimestamp() + " FATAL: method '" + name + "' NOT FOUND IN ISSM ** ");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public void setXboxController(CommandXboxController xb) {
        xboxController = xb;
        System.out.println("Set Xbox Controller");
    }

    public void setCurrentState(ISState newState){
        if (currentState != newState ) {
            System.out.println(" ** " + Timer.getFPGATimestamp() + " ISSM: " + periodicRunCounter + " : " + currentState + " (" + currentInput + ") --> " + newState + " ** ");
        }
        currentState = newState;
        if(currentState == ISState.ALL_STOP){
            setAllStop();
        }
    }

    public synchronized void setCurrentInput(ISInput newInput){
        // if (currentInput != newInput) {
        //     System.out.println(" ** " + Timer.getFPGATimestamp() + " ISSM: " + periodicRunCounter + " INPUT: " + newInput + " ** ");
        // }
        currentInput = newInput;
        run();
    }

    Object STATE_TRANSITION_TABLE[][] = {
    //   CURRENT                          INPUT                                 OPERATION                     NEXT
        {ISState.ALL_STOP,                ISInput.START_INTAKE,                 "startIntake",                ISState.INTAKING},
        {ISState.ALL_STOP,                ISInput.START_JIGGLE,                 "startSpinDownShooter",       ISState.SPIN_DOWN_SHOOTER},
        {ISState.ALL_STOP,                ISInput.START_SPEAKER,                "startSpinUpShooter",         ISState.SPIN_UP_SHOOTER},    
        {ISState.ALL_STOP,                ISInput.START_TRAP,                   "startShootTrap",             ISState.SHOOTING_AT_TRAP}, 
        {ISState.ALL_STOP,                ISInput.START_AMP,                    "startShootAmp",              ISState.SHOOTING_AT_AMP},    
        {ISState.ALL_STOP,                ISInput.START_EJECT,                  "startEject",                 ISState.EJECTING},
        {ISState.ALL_STOP,                ISInput.START_SHOOT_INTAKE,           "startIntakeFromShoot",       ISState.INTAKE_SHOOTER_WAIT},
        {ISState.ALL_STOP,                ISInput.JUST_SHOOT,                   "startShootSpeaker",          ISState.SHOOTING_AT_SPEAKER},
        {ISState.INTAKING_NO_JIGGLE,      ISInput.JUST_SHOOT,                   "startShootSpeaker",          ISState.SHOOTING_AT_SPEAKER},
        {ISState.READY_TO_SHOOT,          ISInput.JUST_SHOOT,                   "startShootSpeaker",          ISState.SHOOTING_AT_SPEAKER}, // redundant (covered by START_SPEAKER)
        {ISState.SPIN_UP_SHOOTER,         ISInput.JUST_SHOOT,                   "startShootSpeaker",          ISState.SHOOTING_AT_SPEAKER},
        {ISState.SPIN_UP_SHOOTER,         ISInput.START_EJECT,                   "startEject",          ISState.EJECTING},
        {ISState.ALL_STOP,                ISInput.INTAKE_NO_JIGGLE,             "startIntakeNoJiggle",        ISState.INTAKING_NO_JIGGLE},
        {ISState.INTAKING_NO_JIGGLE,      ISInput.FORWARD_LIMIT_REACHED,        "turnOnLED",                  ISState.READY_TO_SHOOT},
        {ISState.INTAKING_NO_JIGGLE,      ISInput.STOP_INTAKE,                  "setAllStop",                 ISState.ALL_STOP},
        {ISState.INTAKE_SHOOTER_WAIT,     ISInput.HAS_NOTE,                     "startIFRHasNote",            ISState.INTAKE_SHOOTER_HAS_NOTE}, 
        {ISState.INTAKE_SHOOTER_HAS_NOTE, ISInput.NOTE_SETTLED,                 "startSpinDownShooter",       ISState.SPIN_DOWN_SHOOTER},      
        {ISState.INTAKING,                ISInput.STOP_INTAKE,                  "setAllStop",                 ISState.ALL_STOP},
        {ISState.INTAKING,                ISInput.FORWARD_LIMIT_REACHED,        "startSpinDownShooter",       ISState.SPIN_DOWN_SHOOTER},
        {ISState.SPIN_DOWN_SHOOTER,       ISInput.SHOOTER_READY_FOR_JIGGLE,     "startJiggleUp",              ISState.JIGGLING_UP},
        {ISState.JIGGLING_UP,             ISInput.JIGGLE_UP_TIMER_EXPIRED,      "startJiggleDown",            ISState.JIGGLING_DOWN},
        {ISState.JIGGLING_DOWN,           ISInput.JIGGLE_DOWN_NOTE_SETTLED,     "startSpinUpShooter",         ISState.SPIN_UP_SHOOTER},
        {ISState.SPIN_UP_SHOOTER,         ISInput.SHOOTER_UP_TO_SPEED,          "doNothing",                  ISState.READY_TO_SHOOT},
        {ISState.SPIN_UP_SHOOTER,         ISInput.STOP_SPEAKER,                 "setAllStop",                 ISState.ALL_STOP},  
        {ISState.READY_TO_SHOOT,          ISInput.START_SPEAKER,                "startShootSpeaker",          ISState.SHOOTING_AT_SPEAKER},
        {ISState.READY_TO_SHOOT,          ISInput.START_AMP,                    "startShootAmp",              ISState.SHOOTING_AT_AMP},
        {ISState.READY_TO_SHOOT,          ISInput.START_TRAP,                   "startShootTrap",             ISState.SHOOTING_AT_TRAP},
        {ISState.READY_TO_SHOOT,          ISInput.START_JIGGLE,                 "startSpinDownShooter",       ISState.SPIN_DOWN_SHOOTER},
        {ISState.READY_TO_SHOOT,          ISInput.START_EJECT,                  "startEject",                 ISState.EJECTING},
        {ISState.SHOOTING_AT_SPEAKER,     ISInput.STOP_SPEAKER,                 "setAllStop",                 ISState.ALL_STOP},
        {ISState.SHOOTING_AT_AMP,         ISInput.STOP_AMP,                     "setAllStop",                 ISState.ALL_STOP},
        {ISState.SHOOTING_AT_TRAP,        ISInput.STOP_TRAP,                    "setAllStop",                 ISState.ALL_STOP},
        {ISState.EJECTING,                ISInput.STOP_EJECT,                   "setAllStop",                 ISState.ALL_STOP},

        {ISState.SPIN_UP_SHOOTER,          ISInput.START_LOBSHOT,               "getReadyForLobShot",         ISState.SPIN_SHOOTER_TO_LOB},
        {ISState.READY_TO_SHOOT,           ISInput.START_LOBSHOT,               "getReadyForLobShot",         ISState.SPIN_SHOOTER_TO_LOB},
        {ISState.SPIN_SHOOTER_TO_LOB,      ISInput.SHOOTER_AT_LOB_SPEED,        "doNothing",                  ISState.READY_TO_LOB},
        {ISState.READY_TO_LOB,             ISInput.START_SPEAKER,               "startLobShot",               ISState.LOBBING},
        {ISState.LOBBING,                  ISInput.STOP_SPEAKER,                "setAllStop",                 ISState.ALL_STOP},
        {ISState.LOBBING,                  ISInput.STOP_LOBSHOT,                "cancelLob",                 ISState.ALL_STOP},
        {ISState.SPIN_SHOOTER_TO_LOB,      ISInput.STOP_LOBSHOT,                "cancelLob",                ISState.ALL_STOP},
        {ISState.READY_TO_LOB,             ISInput.STOP_LOBSHOT,                "cancelLob",                 ISState.ALL_STOP},
        {ISState.READY_TO_LOB,             ISInput.STOP_SPEAKER,                "setAllStop",                 ISState.ALL_STOP},
        {ISState.SPIN_SHOOTER_TO_LOB,     ISInput.STOP_SPEAKER,                "setAllStop",                ISState.ALL_STOP},
    };
    
    private void setInputs() {
        if(currentState == ISState.INTAKING && m_intakeSubsystem.forwardLimitReached()){
            setCurrentInput(ISInput.FORWARD_LIMIT_REACHED);
        }
        if(currentState == ISState.JIGGLING_UP && Timer.getFPGATimestamp() - jiggleUpTimerStarted > JIGGLE_UP_TIMER_SECONDS){
            setCurrentInput(ISInput.JIGGLE_UP_TIMER_EXPIRED);
        }
        if(currentState == ISState.JIGGLING_DOWN && ((Timer.getFPGATimestamp() - jiggleDownTimerStarted > JIGGLE_DOWN_TIMER_SECONDS) || m_intakeSubsystem.noteSettled())){
           setCurrentInput(ISInput.JIGGLE_DOWN_NOTE_SETTLED);
        }
        if(currentState == ISState.SPIN_UP_SHOOTER && (m_shooterSubsystem.getShooterVelocity() > 85)){
            setCurrentInput(ISInput.SHOOTER_UP_TO_SPEED);
        }
        if(currentState == ISState.SPIN_DOWN_SHOOTER && (m_shooterSubsystem.getShooterVelocity()  < -50/60)){
            setCurrentInput(ISInput.SHOOTER_READY_FOR_JIGGLE);
        }
        if(currentState == ISState.INTAKE_SHOOTER_WAIT && (m_intakeSubsystem.hasNote())){
            setCurrentInput(ISInput.HAS_NOTE);
        }
        if(currentState == ISState.INTAKE_SHOOTER_HAS_NOTE && (m_intakeSubsystem.noteSettled())){
            setCurrentInput(ISInput.NOTE_SETTLED);
        }
        if(currentState == ISState.SPIN_SHOOTER_TO_LOB && (Math.abs(m_shooterSubsystem.getShooterVelocity() - LOBSPEED ) <  2)){
            setCurrentInput(ISInput.SHOOTER_AT_LOB_SPEED);
        }
    }

    public void periodic() { 
        periodicRunCounter = 0;
        double startTime = Timer.getFPGATimestamp();
        run();
        double elapsedMs = (int)((Timer.getFPGATimestamp() - startTime) * 1e3);
        if(elapsedMs >= 10){
            System.out.println(" ** ISSM ran for " + elapsedMs + " ms! ** ");
        }
        if(periodicRunCounter > 3){
            System.out.println(" ** ISSM run() method ran " + periodicRunCounter + " times! ** ");
        }
        if(Robot.doSD()){
            SmartDashboard.putBoolean("noteSettled", m_intakeSubsystem.noteSettled());
            SmartDashboard.putBoolean("hasNote", m_intakeSubsystem.hasNote());
        }

        if ((currentState == ISState.SPIN_SHOOTER_TO_LOB ) || (currentState == ISState.READY_TO_LOB )) {
            m_ledSubsystem.setColor(LedOption.YELLOW);
        } else if (haveNote|| !robotStarted) {
            turnOnLED();
        } else {
            m_ledSubsystem.setColor(LedOption.BLACK);
        }
    }

    public void setInitialState(ISState initialState){
        setCurrentState(initialState);
    }

    public boolean setAllStop(){
        m_shooterSubsystem.stopShooting();
        m_intakeSubsystem.intakeState(0.0);
        m_intakeSubsystem.feedState(0.0);
        m_intakeSubsystem.enableLimitSwitch();
        m_intakeSubsystem.enableReverseLimitSwitch();
        if (xboxController != null) {
            xboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
        }
        return true;
    }

    public boolean cancelLob(){
        m_shooterSubsystem.shoot();
        m_intakeSubsystem.intakeState(-0.5);
        m_intakeSubsystem.feedState(0.0);
        m_intakeSubsystem.enableLimitSwitch();
        m_intakeSubsystem.enableReverseLimitSwitch();
        return true;
    }

    public boolean startIntake(){
        m_shooterSubsystem.reverseSlow();
        m_intakeSubsystem.intakeState(1.0);
        m_intakeSubsystem.feedState(1.0);
        m_intakeSubsystem.enableLimitSwitch();
        m_intakeSubsystem.enableReverseLimitSwitch();
        return true;
    }

    public boolean startIntakeNoJiggle(){
        m_shooterSubsystem.shoot();
        m_intakeSubsystem.intakeState(1.0);
        m_intakeSubsystem.feedState(1.0);
        m_intakeSubsystem.enableLimitSwitch();
        m_intakeSubsystem.enableReverseLimitSwitch();
        return true;
    }

    public boolean startJiggleUp(){
        m_shooterSubsystem.reverseSlow();
        m_intakeSubsystem.intakeState(-0.5);
        m_intakeSubsystem.feedState(0.5);
        m_intakeSubsystem.disableLimitSwitch();
        m_intakeSubsystem.enableReverseLimitSwitch();
        jiggleUpTimerStarted = Timer.getFPGATimestamp();
        return true;
    }

    public boolean startSpinDownShooter(){
        m_shooterSubsystem.reverseSlow();
        m_intakeSubsystem.intakeState(-0.5);
        m_intakeSubsystem.feedState(0.0);
        m_intakeSubsystem.disableLimitSwitch();
        m_intakeSubsystem.enableReverseLimitSwitch();
        haveNote = true;
        if (xboxController != null) {
            xboxController.getHID().setRumble(RumbleType.kBothRumble, 1);
        }
        return true;
    }

    public boolean getReadyForLobShot(){
        System.out.println("getting ready for lobshot with speed " + LOBSPEED);
        m_shooterSubsystem.lobShot(LOBSPEED);
        m_intakeSubsystem.intakeState(-0.5);
        m_intakeSubsystem.feedState(0.0);
        m_intakeSubsystem.disableLimitSwitch();
        m_intakeSubsystem.enableReverseLimitSwitch();
        haveNote = true;
        return true;
    }

    public boolean startLobShot(){
        System.out.println("starting lobshot with speed " + LOBSPEED);

        m_shooterSubsystem.lobShot(LOBSPEED);
        m_intakeSubsystem.intakeState(-0.5);
        m_intakeSubsystem.feedState(1.0);
        m_intakeSubsystem.disableLimitSwitch();
        m_intakeSubsystem.enableReverseLimitSwitch();
        haveNote = false;
        return true;
    }

    public boolean startJiggleDown(){
        m_shooterSubsystem.reverseSlow();
        m_intakeSubsystem.intakeState(-0.5);
        m_intakeSubsystem.feedState(-0.5);
        m_intakeSubsystem.enableLimitSwitch();
        m_intakeSubsystem.enableReverseLimitSwitch();
        jiggleDownTimerStarted = Timer.getFPGATimestamp();
        return true;
    }

    public boolean startSpinUpShooter(){
        m_shooterSubsystem.shoot();
        m_intakeSubsystem.intakeState(-0.5);
        m_intakeSubsystem.feedState(1.0);
        m_intakeSubsystem.enableLimitSwitch();
        m_intakeSubsystem.enableReverseLimitSwitch();
        if (xboxController != null) {
            xboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
        }
        return true;
    }

    public boolean startShootSpeaker(){
        m_shooterSubsystem.shoot();
        m_intakeSubsystem.intakeState(-0.5);
        m_intakeSubsystem.feedState(1.0);
        m_intakeSubsystem.disableLimitSwitch();
        m_intakeSubsystem.enableReverseLimitSwitch();
        haveNote = false;
        return true;
    }

    public boolean startShootAmp(){
        m_shooterSubsystem.shootAmp();
        m_intakeSubsystem.intakeState(-0.5);
        m_intakeSubsystem.feedState(1.0);
        m_intakeSubsystem.disableLimitSwitch();
        m_intakeSubsystem.enableReverseLimitSwitch();
        haveNote = false;
        return true;
    }

    public boolean startShootTrap(){
        m_shooterSubsystem.stopShooting();
        m_intakeSubsystem.intakeState(-0.6);
        m_intakeSubsystem.feedState(-0.6);
        m_intakeSubsystem.enableLimitSwitch();
        m_intakeSubsystem.disableReverseLimitSwitch();
        haveNote = false;
        return true;
    }

    public boolean startEject(){
        m_shooterSubsystem.stopShooting();
        m_intakeSubsystem.intakeState(-1.0);
        m_intakeSubsystem.feedState(-1.0);
        m_intakeSubsystem.enableLimitSwitch();
        m_intakeSubsystem.disableReverseLimitSwitch();
        haveNote = false;
        return true;
    }

    public boolean startIntakeFromShoot(){
        m_shooterSubsystem.shooterAsIntake();
        m_intakeSubsystem.intakeState(-0.2);
        m_intakeSubsystem.feedState(-0.2);
        m_intakeSubsystem.enableLimitSwitch();
        m_intakeSubsystem.disableReverseLimitSwitch();
        return true;
    }
    
   public boolean startIFRHasNote(){
        m_shooterSubsystem.shooterAsIntake();
        m_intakeSubsystem.intakeState(-0.2);
        m_intakeSubsystem.feedState(-0.2);
        m_intakeSubsystem.enableLimitSwitch();
        m_intakeSubsystem.enableReverseLimitSwitch();
        return true;
    }

    public boolean turnOnLED(){
        m_ledSubsystem.setColor(m_visionSubsystem.isConfident() ? LedOption.WHITE : LedOption.RED);
        return true;
    }

    public void run(){
        periodicRunCounter++;
        Object[] operationAndNextState = lookupOperationAndNextState(currentState, currentInput);
        if(operationAndNextState != null){
            String operation = (String) operationAndNextState[0];
            ISState nextState = (ISState) operationAndNextState[1];

            if (nextState == null){
                System.out.println (" ** " + Timer.getFPGATimestamp() + " FATAL: ISSM nextState is NULL ** ");
            }
            else{
                Method method = methods.get(operation);
                if(method != null){
                    try {
                        System.out.println(" ** " + Timer.getFPGATimestamp() + " ISSM: " + periodicRunCounter + " running: " + operation + " ** ");
                        if((Boolean)method.invoke(this) && nextState != null){
                            setCurrentState(nextState);
                        }
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
                else{
                    System.err.println(" ** " + Timer.getFPGATimestamp() + " FATAL: ISSM METHOD FOR OPERATION: " + operation + "() HAS NOT BEEN IMPLEMENTED! ** ");
                }
            }
        }
        setInputs();
    }

    private Object[] lookupOperationAndNextState(ISState currentState, ISInput currentInput) {
        if(currentState != null && currentInput != null){
            for(Object[] transition : STATE_TRANSITION_TABLE){
                ISState state = (ISState) transition[0];
                ISInput input = (ISInput) transition[1];
                String oper = (String) transition[2];
                ISState next = (ISState) transition[3];
                if(state == currentState && input == currentInput){
                    return new Object[]{oper, next};
                }
            }
        }
        return null;
    }

    public boolean doNothing(){
        return true;
    }

    public void startLEDs(){
        robotStarted = true;
    }

     public void setLobSpeed(double LobSpeed) {
        LOBSPEED = LobSpeed;
        System.out.println("Setting lobspeed to " + LOBSPEED);
     }
}
