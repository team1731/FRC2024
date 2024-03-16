/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ISInput;
import frc.robot.IntakeShootStateMachine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command to fire into the speaker
 */
public class ScoreAmpAndRetractReverseCommand extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final IntakeShootStateMachine m_intakeShootStateMachine;
	private final ElevatorSubsystem m_elevatorSubsystem;
	private final WristSubsystem m_wristSubsystem;
    private double ampTimeStarted;


	/**
	 * Creates a new Fire into the speaker
	 *
	 * @param IntakeSubsystem     
	 * @param seqSubsystem        
	 * @param PoseEstimatorSubsystem 
	 * @param ElevatorSubsystem
	 * @param WristSubsystem 
	 */
	public ScoreAmpAndRetractReverseCommand(IntakeShootStateMachine intakeShootStateMachine,  ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
		m_intakeShootStateMachine = intakeShootStateMachine;
		m_elevatorSubsystem = elevatorSubsystem;
		m_wristSubsystem = wristSubsystem;



		// Use addRequirements() here to declare subsystem dependencies.
		if ( elevatorSubsystem != null && wristSubsystem != null) {
			addRequirements( elevatorSubsystem, wristSubsystem);
		}
	}

	// Called when the command is initially scheduled.
	// If it is used as Default command then it gets call all the time
	@Override
	public void initialize() {
	//	m_wristSubsystem.moveWrist(Constants.WristConstants.wristAmpReversePosition);
	//	m_elevatorSubsystem.moveElevator(Constants.ElevatorConstants.elevatorAmpReversePosition);
	    m_intakeShootStateMachine.setCurrentInput(ISInput.START_AMP);
	   // m_shooterSubsystem.shootAmp();
		ampTimeStarted = Timer.getFPGATimestamp();

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		if (Timer.getFPGATimestamp() - ampTimeStarted > .25) {
			m_intakeShootStateMachine.setCurrentInput(ISInput.STOP_AMP);
		  }

		
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
			m_wristSubsystem.moveWrist(Constants.WristConstants.wristHomePosition);
			m_elevatorSubsystem.moveElevator(Constants.ElevatorConstants.elevatorHomePosition);
			m_wristSubsystem.retractTrapFlap();
			m_intakeShootStateMachine.setCurrentInput(ISInput.STOP_AMP);
			
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		if (Timer.getFPGATimestamp() - ampTimeStarted > .25) {
			m_intakeShootStateMachine.setCurrentInput(ISInput.STOP_AMP);
			return true;
		} else {
			return false;
		}

	}
}