package frc.robot;

import java.lang.reflect.Method;
import java.util.HashMap;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

enum State {
    START_CONFIG,
    ELEVATOR_ABOVE_CHAIN,
    ROBOT_LATCHED_ON_CHAIN,
    ELEVATOR_AT_TRAP,
    LOWER_ELEVATOR,
    END
}

enum Input {
    BEGIN,
    STARTING,
    ELEVATOR_ABOVE_CHAIN,
    ROBOT_LATCHED_ON_CHAIN,
    ELEVATOR_AT_TRAP,
    TIMER_HAS_EXPIRED,
    ABORT
}

public class ClimbStateMachine {
	private final IntakeSubsystem m_intakeSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
	private final ElevatorSubsystem m_elevatorSubsystem;
	private final WristSubsystem m_wristSubsystem;
    private State currentState;
    private Input currentInput;
    private HashMap<String, Method> methods;
    private double timerStarted;
    private double NOTE_TIMER_SECONDS = .5;

    public ClimbStateMachine(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem,  ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem){
        m_intakeSubsystem = intakeSubsystem;
		m_shooterSubsystem = shooterSubsystem;
		m_elevatorSubsystem = elevatorSubsystem;
		m_wristSubsystem = wristSubsystem;

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

    public void setCurrentState(State newState){
        System.out.println("\n\n\n" + Timer.getFPGATimestamp() + " $$$$$$$$$$$$$$$   CLIMB STATE MACHINE ========> SETTING STATE: " + newState + " $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n\n\n");
        currentState = newState;
    }

    public void setCurrentInput(Input newInput){
        System.out.println("\n\n\n" + Timer.getFPGATimestamp() + " >>>>>>>>>>>>>>>   CLIMB STATE MACHINE ========> SETTING INPUT: " + newInput + " <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n\n\n");
        currentInput = newInput;
    }

    Object STATE_TRANSITION_TABLE[][] = {
      // CURRENT                        INPUT                               OPERATION                     NEXT
        {State.START_CONFIG,            Input.BEGIN,                        "raiseElevatorAboveChain",    State.ELEVATOR_ABOVE_CHAIN},
        {State.ELEVATOR_ABOVE_CHAIN,    Input.ELEVATOR_ABOVE_CHAIN,         "latchRobotOnChain",          State.ROBOT_LATCHED_ON_CHAIN},
        {State.ROBOT_LATCHED_ON_CHAIN,  Input.STARTING,                     "raiseElevatorToTrap",        State.ELEVATOR_AT_TRAP},
        {State.ELEVATOR_AT_TRAP,        Input.ELEVATOR_AT_TRAP,             "ejectNote",                  State.LOWER_ELEVATOR},
        {State.ELEVATOR_AT_TRAP,        Input.ABORT,                        "getOffTheLedge",             State.ROBOT_LATCHED_ON_CHAIN},
        {State.LOWER_ELEVATOR,          Input.TIMER_HAS_EXPIRED,            "getOffTheLedge",             State.ROBOT_LATCHED_ON_CHAIN},
        {State.LOWER_ELEVATOR,          Input.ABORT,                        "getOffTheLedge",             State.ROBOT_LATCHED_ON_CHAIN},
        {State.END,                     Input.BEGIN,                        "lowerRobotDown",             State.START_CONFIG}
    };

    public void setInput(){
        if(currentState == State.ELEVATOR_AT_TRAP && m_elevatorSubsystem.isAtPosition(Constants.ElevatorConstants.elevatorTrapPosition)){
            setCurrentInput(Input.ELEVATOR_AT_TRAP);
        }
        if(currentState == State.LOWER_ELEVATOR && Timer.getFPGATimestamp() - timerStarted > NOTE_TIMER_SECONDS){
            setCurrentInput(Input.TIMER_HAS_EXPIRED);
        }
    }

    public void setInitialState(State initialState){
        setCurrentState(initialState);
    }

    public void run(){
        Object[] operationAndNextState = lookupOperationAndNextState(currentState, currentInput);
        if(operationAndNextState != null){
            String operation = (String) operationAndNextState[0];
            State nextState = (State) operationAndNextState[1];
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

    private Object[] lookupOperationAndNextState(State currentState, Input currentInput) {
        if(currentState != null && currentInput != null){
            for(Object[] transition : STATE_TRANSITION_TABLE){
                State state = (State) transition[0];
                Input input = (Input) transition[1];
                String oper = (String) transition[2];
                State next = (State) transition[3];
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
        m_intakeSubsystem.trapFeed();
        timerStarted = Timer.getFPGATimestamp();
        return true;
    }

    public boolean getOffTheLedge(){
        m_wristSubsystem.retractTrapFlap();
        m_wristSubsystem.moveWristSlow(WristConstants.wristAmpReversePosition, WristConstants.MMVelSlow);
        m_elevatorSubsystem.moveElevator(Constants.ElevatorConstants.elevatorHomePosition);
        m_intakeSubsystem.stoptrapFeed();
        return true;
    }

    public boolean doNothing(){
        return true;
    }

    public boolean lowerRobotDown(){
        return true;
    }

    public void setInputStarting() {
        setCurrentInput(Input.STARTING);
    }

    public void setInputAbort() {
        setCurrentInput(Input.ABORT);
    }
}
