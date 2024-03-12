package frc.robot;

import java.lang.reflect.Method;
import java.util.HashMap;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

enum ISState {
    ALL_STOP,
    INTAKING,
    JIGGLING_UP,
    JIGGLING_DOWN,
    SHOOTING_AT_SPEAKER,
    SHOOTING_AT_AMP,
    PREPARE_TO_SHOOT,
    EJECTING
}

enum ISInput {
    START_INTAKE,
    STOP_INTAKE,
    START_AMP,
    STOP_AMP,
    START_SPEAKER,
    STOP_SPEAKER,
    START_JIGGLE,
    STOP_JIGGLE,
    START_EJECT,
    STOP_EJECT,
    FORWARD_LIMIT_REACHED,
    JIGGLE_UP_TIMER_EXPIRED
}

public class IntakeShootStateMachine {
	private final IntakeSubsystem m_intakeSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
	private final WristSubsystem m_wristSubsystem;
    private ISState currentState;
    private ISInput currentInput;
    private HashMap<String, Method> methods;
    private double jiggleUpTimerStarted;
    private double JIGGLE_UP_TIMER_SECONDS = 0.5;

    public IntakeShootStateMachine(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, WristSubsystem wristSubsystem){
        m_intakeSubsystem = intakeSubsystem;
		m_shooterSubsystem = shooterSubsystem;
		m_wristSubsystem = wristSubsystem;

        methods = new HashMap<String, Method>();
        for(Object[] transition : STATE_TRANSITION_TABLE){
            String name = (String) transition[2];
            try {
                Method method = IntakeShootStateMachine.class.getMethod(name);
                if(method != null){
                    methods.put(name, method);
                }
            } catch (NoSuchMethodException e) {
                System.out.println("\n\n\n\n" + Timer.getFPGATimestamp() + " ERROR: method '" + name + "' NOT FOUND IN CLASS IntakeShootStateMachine!\n\n\n\n");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public void setCurrentState(ISState newState){
        System.out.println("\n\n\n" + Timer.getFPGATimestamp() + " $$$$$$$$$$$$$$$   INTAKE/SHOOT STATE MACHINE ========> SETTING STATE: " + newState + " $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n\n\n");
        currentState = newState;
        if(currentState == ISState.ALL_STOP){
            allStop();
        }
    }

    public void setCurrentInput(ISInput newInput){
        System.out.println("\n\n\n" + Timer.getFPGATimestamp() + " >>>>>>>>>>>>>>>   INTAKE/SHOOT STATE MACHINE ========> SETTING INPUT: " + newInput + " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n\n");
        currentInput = newInput;
    }

    Object STATE_TRANSITION_TABLE[][] = {
    //   CURRENT                          INPUT                                 OPERATION                     NEXT
        {ISState.ALL_STOP,                ISInput.START_INTAKE,                 "startIntake",                ISState.INTAKING},
        {ISState.INTAKING,                ISInput.START_INTAKE,                 "doNothing",                  ISState.INTAKING},
        {ISState.INTAKING,                ISInput.STOP_INTAKE,                  "allStop",                    ISState.ALL_STOP},
        {ISState.INTAKING,                ISInput.FORWARD_LIMIT_REACHED,        "jiggleUp",                   ISState.JIGGLING_UP},
        {ISState.JIGGLING_UP,             ISInput.START_INTAKE,                 "doNothing",                  ISState.JIGGLING_UP},
        {ISState.JIGGLING_UP,             ISInput.JIGGLE_UP_TIMER_EXPIRED,      "jiggleDown",                 ISState.JIGGLING_DOWN},

    };


    public void setInput(){
        if(currentState == ISState.INTAKING && m_intakeSubsystem.forwardLimitReached()){
            setCurrentInput(ISInput.FORWARD_LIMIT_REACHED);
        }
        if(currentState == ISState.JIGGLING_UP && Timer.getFPGATimestamp() - jiggleUpTimerStarted > JIGGLE_UP_TIMER_SECONDS){
            setCurrentInput(ISInput.JIGGLE_UP_TIMER_EXPIRED);
        }
    }

    public void setInitialState(ISState initialState){
        setCurrentState(initialState);
    }

    public boolean allStop(){
        m_shooterSubsystem.stopShooting();
        m_intakeSubsystem.stopIntake();
        m_intakeSubsystem.stopFeed();
        m_intakeSubsystem.enableLimitSwitch();
        m_intakeSubsystem.enableReverseLimitSwitch();
        return true;
    }

    public boolean startIntake(){
        m_shooterSubsystem.reverseSlow();
        m_intakeSubsystem.intake(1.0);
        m_intakeSubsystem.feed(1.0);
        m_intakeSubsystem.enableLimitSwitch();
        m_intakeSubsystem.enableReverseLimitSwitch();
        return true;
    }

    public boolean jiggleUp(){
        m_shooterSubsystem.reverseSlow();
        m_intakeSubsystem.intake(-0.5);
        m_intakeSubsystem.feed(0.5);
        m_intakeSubsystem.disableLimitSwitch();
        m_intakeSubsystem.enableReverseLimitSwitch();
        jiggleUpTimerStarted = Timer.getFPGATimestamp();
        return true;
    }

    public void run(){
        Object[] operationAndNextState = lookupOperationAndNextState(currentState, currentInput);
        if(operationAndNextState != null){
            String operation = (String) operationAndNextState[0];
            ISState nextState = (ISState) operationAndNextState[1];
            Method method = methods.get(operation);
            if(method != null){
                try {
                    System.out.println("\n\n\n" + Timer.getFPGATimestamp() + " >>>>>>>>>>>>>>>   INTAKE/SHOOT STATE MACHINE ========> RUNNING: " + operation + " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n\n");
                    if((Boolean)method.invoke(this) && nextState != null){
                        setCurrentState(nextState);
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            else{
                System.err.println("\n\n" + Timer.getFPGATimestamp() + " FATAL: INTAKE/SHOOT STATE MACHINE OPERATION NOT FOUND: " + operation + "() !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
            }
        }
        setInput();
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

    public boolean lowerRobotDown(){
        return true;
    }
}
