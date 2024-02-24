package frc.robot;

import java.lang.reflect.Method;
import java.util.HashMap;

enum State {
    START_CONFIG,
    ELEVATOR_ABOVE_CHAIN,
    ROBOT_LATCHED_ON_CHAIN,
    ELEVATOR_AT_TRAP,
    END
}

enum Input {
    BEGIN,
    ELEVATOR_ABOVE_CHAIN,
    ROBOT_LATCHED_ON_CHAIN,
    ELEVATOR_AT_TRAP
}

public class ClimbStateMachine {
    private State currentState;
    private Input currentInput;
    private HashMap<String, Method> methods;

    public ClimbStateMachine(State initialState){
        currentState = initialState;
        for(Object[] transition : STATE_TRANSITION_TABLE){
            String name = (String) transition[2];
            try {
                Method method = ClimbStateMachine.class.getMethod(name);
                methods.put(name, method);
            } catch (NoSuchMethodException e) {
                System.out.println("ERROR: method '" + name + "' NOT FOUND IN CLASS ClimbStateMachine!");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public void setCurrentInput(Input newInput){
        currentInput = newInput;
    }

    Object STATE_TRANSITION_TABLE[][] = {
      // CURRENT                        INPUT                               OPERATION                     NEXT
        {State.START_CONFIG,            Input.BEGIN,                        "raiseElevatorAboveChain",    State.ELEVATOR_ABOVE_CHAIN},
        {State.ELEVATOR_ABOVE_CHAIN,    Input.ELEVATOR_ABOVE_CHAIN,         "latchRobotOnChain",          State.ROBOT_LATCHED_ON_CHAIN},
        {State.ROBOT_LATCHED_ON_CHAIN,  Input.ROBOT_LATCHED_ON_CHAIN,       "raiseElevatorToTrap",        State.ELEVATOR_AT_TRAP},
        {State.ELEVATOR_AT_TRAP,        Input.ELEVATOR_AT_TRAP,             "doNothing",                  State.END},
        {State.END,                     Input.BEGIN,                        "lowerRobotDown",             State.START_CONFIG}
    };

    public void run(){
        Object[] operationAndNextState = lookupOperationAndNextState(currentState, currentInput);
        String operation = (String) operationAndNextState[0];
        State nextState = (State) operationAndNextState[1];
        if(nextState != null && nextState != currentState){
            Method method = methods.get(operation);
            if(method != null){
                try {
                    System.out.println("========> RUNNING: " + operation);
                    if((Boolean)method.invoke(this)){
                        currentState = nextState;
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private Object[] lookupOperationAndNextState(State currentState, Input currentInput) {
        for(Object[] transition : STATE_TRANSITION_TABLE){
            State state = (State) transition[0];
            Input input = (Input) transition[1];
            String oper = (String) transition[2];
            State next = (State) transition[3];
            if(state == currentState && input == currentInput){
                return new Object[]{oper, next};
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
        return true;
    }

    public boolean doNothing(){
        return true;
    }

    public boolean lowerRobotDown(){
        return true;
    }
}
