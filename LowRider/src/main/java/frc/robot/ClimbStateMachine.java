package frc.robot;

import java.lang.reflect.Method;
import java.util.HashMap;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

enum CState {
    START_CONFIG,
    ELEVATOR_ABOVE_CHAIN,
    ROBOT_LATCHED_ON_CHAIN,
    ELEVATOR_AT_TRAP,
    LOWER_ELEVATOR,
    END
}

enum CInput {
    BEGIN,
    STARTING,
    ELEVATOR_ABOVE_CHAIN,
    ROBOT_LATCHED_ON_CHAIN,
    ELEVATOR_AT_TRAP,
    TIMER_HAS_EXPIRED,
    ABORT
}

public class ClimbStateMachine {
	private final ElevatorSubsystem m_elevatorSubsystem;
	private final WristSubsystem m_wristSubsystem;
    private final IntakeShootStateMachine m_intakeShootStateMachine;
    private CState currentState;
    private CInput currentInput;
    private HashMap<String, Method> methods;
    private double timerStarted;
    private double NOTE_TIMER_SECONDS = 0.5;

    public ClimbStateMachine( ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, IntakeShootStateMachine intakeShootStateMachine){
		m_elevatorSubsystem = elevatorSubsystem;
		m_wristSubsystem = wristSubsystem;
        m_intakeShootStateMachine = intakeShootStateMachine;

        methods = new HashMap<String, Method>();
        for(Object[] transition : STATE_TRANSITION_TABLE){
            String name = (String) transition[2];
            try {
                Method method = ClimbStateMachine.class.getMethod(name);
                if(method != null){
                    methods.put(name, method);
                }
            } catch (NoSuchMethodException e) {
                System.out.println("\n\n\n\n" + Timer.getFPGATimestamp() + " ERROR: method '" + name + "' NOT FOUND IN CLASS ClimbStateMachine!\n\n\n\n");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public void setCurrentState(CState newState){
        System.out.println("\n\n\n" + Timer.getFPGATimestamp() + " $$$$$$$$$$$$$$$   CLIMB STATE MACHINE ========> SETTING STATE: " + newState + " $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n\n\n");
        currentState = newState;
    }

    public void setCurrentInput(CInput newInput){
        System.out.println("\n\n\n" + Timer.getFPGATimestamp() + " >>>>>>>>>>>>>>>   CLIMB STATE MACHINE ========> SETTING INPUT: " + newInput + " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n\n");
        currentInput = newInput;
    }

    Object STATE_TRANSITION_TABLE[][] = {
      // CURRENT                        INPUT                               OPERATION                     NEXT
        {CState.START_CONFIG,            CInput.BEGIN,                        "raiseElevatorAboveChain",    CState.ELEVATOR_ABOVE_CHAIN},
        {CState.ELEVATOR_ABOVE_CHAIN,    CInput.ELEVATOR_ABOVE_CHAIN,         "latchRobotOnChain",          CState.ROBOT_LATCHED_ON_CHAIN},
        {CState.ROBOT_LATCHED_ON_CHAIN,  CInput.STARTING,                     "raiseElevatorToTrap",        CState.ELEVATOR_AT_TRAP},
        {CState.ELEVATOR_AT_TRAP,        CInput.ELEVATOR_AT_TRAP,             "ejectNote",                  CState.LOWER_ELEVATOR},
        {CState.ELEVATOR_AT_TRAP,        CInput.ABORT,                        "getOffTheLedge",             CState.ROBOT_LATCHED_ON_CHAIN},
        {CState.LOWER_ELEVATOR,          CInput.TIMER_HAS_EXPIRED,            "getOffTheLedge",             CState.ROBOT_LATCHED_ON_CHAIN},
        {CState.LOWER_ELEVATOR,          CInput.ABORT,                        "getOffTheLedge",             CState.ROBOT_LATCHED_ON_CHAIN},
        {CState.END,                     CInput.BEGIN,                        "lowerRobotDown",             CState.START_CONFIG}
    };

    public void setInput(){
        if(currentState == CState.ELEVATOR_AT_TRAP && m_elevatorSubsystem.isAtPosition(Constants.ElevatorConstants.elevatorTrapPosition)){
            setCurrentInput(CInput.ELEVATOR_AT_TRAP);
        }
        if(currentState == CState.LOWER_ELEVATOR && Timer.getFPGATimestamp() - timerStarted > NOTE_TIMER_SECONDS){
            setCurrentInput(CInput.TIMER_HAS_EXPIRED);
        }
    }

    public void setInitialState(CState initialState){
        setCurrentState(initialState);
    }

    public void run(){
        Object[] operationAndNextState = lookupOperationAndNextState(currentState, currentInput);
        if(operationAndNextState != null){
            String operation = (String) operationAndNextState[0];
            CState nextState = (CState) operationAndNextState[1];
            Method method = methods.get(operation);
            if(method != null){
                try {
                    System.out.println("\n\n\n" + Timer.getFPGATimestamp() + " >>>>>>>>>>>>>>>   CLIMB STATE MACHINE ========> RUNNING: " + operation + " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n\n");
                    if((Boolean)method.invoke(this) && nextState != null){
                        setCurrentState(nextState);
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
            else{
                System.err.println("\n\n" + Timer.getFPGATimestamp() + " FATAL: STATE MACHINE OPERATION NOT FOUND: " + operation + "() !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");
            }
        }
        setInput();
    }

    private Object[] lookupOperationAndNextState(CState currentState, CInput currentInput) {
        if(currentState != null && currentInput != null){
            for(Object[] transition : STATE_TRANSITION_TABLE){
                CState state = (CState) transition[0];
                CInput input = (CInput) transition[1];
                String oper = (String) transition[2];
                CState next = (CState) transition[3];
                if(state == currentState && input == currentInput){
                    return new Object[]{oper, next};
                }
            }
        }
        return null;
    }

    public boolean raiseElevatorAboveChain(){
        return true;
    }

    public boolean latchRobotOnChain(){
        return true;
    }

    public boolean raiseElevatorToTrap(){
        m_elevatorSubsystem.moveElevator(Constants.ElevatorConstants.elevatorTrapPosition);
        return true;
    }

    public boolean ejectNote(){
        m_intakeShootStateMachine.setCurrentInput(ISInput.START_TRAP);
       // m_intakeSubsystem.trapFeed();
        timerStarted = Timer.getFPGATimestamp();
        return true;
    }

    public boolean getOffTheLedge(){
        m_wristSubsystem.retractTrapFlap();
        m_wristSubsystem.moveWristSlow(WristConstants.wristNewTrapPosition, WristConstants.MMVelSlow);
        m_elevatorSubsystem.moveElevator(0);// To account for slack in chain
        m_intakeShootStateMachine.setCurrentInput(ISInput.STOP_TRAP);
       // m_intakeSubsystem.stoptrapFeed();
        return true;
    }

    public boolean doNothing(){
        return true;
    }

    public boolean lowerRobotDown(){
        return true;
    }

    public void setInputStarting() {
        setCurrentInput(CInput.STARTING);
    }

    public void setInputAbort() {
        setCurrentInput(CInput.ABORT);
    }
}
