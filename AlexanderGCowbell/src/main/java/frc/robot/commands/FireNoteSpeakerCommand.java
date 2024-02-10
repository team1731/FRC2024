/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to fire into the speaker
 */
public class FireNoteSpeakerCommand extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final IntakeSubsystem m_intakeSubsystem;
    private final ShooterSubsystem m_ShooterSubsystem;
	private final PoseEstimatorSubsystem m_poseEstimatorSubsystem;



	/**
	 * Creates a new Fire into the speaker
	 *
	 * @param IntakeSubsystem     
	 * @param seqSubsystem        
	 * @param PoseEstimatorSubsystem 
	 */
	public FireNoteSpeakerCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem,
			PoseEstimatorSubsystem poseEstimatorSubsystem) {
		m_intakeSubsystem = intakeSubsystem;
		m_ShooterSubsystem = shooterSubsystem;
		m_poseEstimatorSubsystem = poseEstimatorSubsystem;

		// Use addRequirements() here to declare subsystem dependencies.
		if (intakeSubsystem != null && shooterSubsystem != null && poseEstimatorSubsystem != null) {
			addRequirements(intakeSubsystem, shooterSubsystem, poseEstimatorSubsystem);
		}
	}

	// Called when the command is initially scheduled.
	// If it is used as Default command then it gets call all the time
	@Override
	public void initialize() {
		
        m_intakeSubsystem.disableLimitSwitch();
		// turn on the shooter if it is not already on
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

        // if we have a good field position, set the elevator and wrist angles based on the distance to the goal
		// optionally take over steering
		// if the elevator and wrist are in range and the shooter is up to speed, run the feeder motor
	    m_intakeSubsystem.feed();



	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_intakeSubsystem.enableLimitSwitch();
        m_intakeSubsystem.stopFeed();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}

}