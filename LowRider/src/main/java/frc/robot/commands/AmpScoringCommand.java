/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

import com.ctre.phoenix.motion.*;


/**
 * Command to fire into the speaker
 */
public class AmpScoringCommand extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final IntakeSubsystem m_intakeSubsystem;
	private final ElevatorSubsystem m_elevatorSubsystem;
	private final WristSubsystem m_wristSubsystem;



	/**
	 * Creates a new Fire into the speaker
	 *
	 * @param IntakeSubsystem     
	 * @param seqSubsystem        
	 * @param PoseEstimatorSubsystem 
	 * @param ElevatorSubsystem
	 * @param WristSubsystem 
	 */
	public AmpScoringCommand(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
		m_intakeSubsystem = intakeSubsystem;
		m_elevatorSubsystem = elevatorSubsystem;
		m_wristSubsystem = wristSubsystem;


		// Use addRequirements() here to declare subsystem dependencies.
		if (intakeSubsystem != null && elevatorSubsystem != null && wristSubsystem != null) {
			addRequirements(intakeSubsystem, elevatorSubsystem, wristSubsystem);
		}
	}

	// Called when the command is initially scheduled.
	// If it is used as Default command then it gets call all the time
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_elevatorSubsystem.sendElevatorUp();
		m_wristSubsystem.moveWrist(Constants.WristConstants.wristAmpPosition);
		if (Math.abs(Constants.ElevatorConstants.elevatorExtendedPosition - m_elevatorSubsystem.getElevatorPosition()) <= Constants.ElevatorConstants.elevatorPositionTolerance){
			m_intakeSubsystem.feed();
		}	
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_elevatorSubsystem.sendElevatorHome();
		m_wristSubsystem.moveWrist(Constants.WristConstants.wristHomePosition);
        m_intakeSubsystem.stopFeed();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}