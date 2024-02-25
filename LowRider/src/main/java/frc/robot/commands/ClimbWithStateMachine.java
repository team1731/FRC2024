/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ClimbStateMachine;

/**
 * Command to fire into the speaker
 */
public class ClimbWithStateMachine extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final ClimbStateMachine m_climbStateMachine;

	/**
	 * Creates a new Fire into the speaker
	 *
	 * @param IntakeSubsystem     
	 * @param seqSubsystem        
	 * @param PoseEstimatorSubsystem 
	 */
	public ClimbWithStateMachine(ClimbStateMachine climbStateMachine) {
		m_climbStateMachine = climbStateMachine;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements();
	}

	// Called when the command is initially scheduled.
	// If it is used as Default command then it gets call all the time
	@Override
	public void initialize() {
		m_climbStateMachine.setInputStarting();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
        m_climbStateMachine.run();
	}


	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_climbStateMachine.setInputAbort();
		m_climbStateMachine.run();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}