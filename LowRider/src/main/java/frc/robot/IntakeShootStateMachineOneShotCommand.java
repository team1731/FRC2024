package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;


public class IntakeShootStateMachineOneShotCommand extends Command {
    private IntakeShootStateMachine intakeShootStateMachine;
    private ISInput input;

    public IntakeShootStateMachineOneShotCommand(IntakeShootStateMachine intakeShootStateMachine, ISInput input){
        this.intakeShootStateMachine = intakeShootStateMachine;
        this.input = input;

    }

	// Called when the command is initially scheduled.
	// If it is used as Default command then it gets call all the time
	@Override
	public void initialize() {
       	 intakeShootStateMachine.setCurrentInput(input);
		
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}
}
