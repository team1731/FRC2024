/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


/**
 * Command to fire into the speaker
 */
public class JiggleCommand extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final IntakeSubsystem m_intakeSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
	private boolean jiggleStarted;
    private boolean allDone= false;




	/**
	 * Creates a new Fire into the speaker
	 *
	 * @param IntakeSubsystem     
	 * @param seqSubsystem        
	 *  
	 */
	public JiggleCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
		m_intakeSubsystem = intakeSubsystem;
		m_shooterSubsystem = shooterSubsystem;
		

		// Use addRequirements() here to declare subsystem dependencies.
		if (intakeSubsystem != null && shooterSubsystem != null) {
			addRequirements(intakeSubsystem, shooterSubsystem);
		}
	}

	// Called when the command is initially scheduled.
	// If it is used as Default command then it gets call all the time
	@Override
	public void initialize() {

	    m_shooterSubsystem.reverseSlow();
		jiggleStarted = false;
		m_intakeSubsystem.initializeJiggle();



	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		//	m_intakeSubsystem.intake(1.0);	
	    if (!jiggleStarted && m_shooterSubsystem.getShooterVelocity() < 0) {
			m_intakeSubsystem.feedUpJiggle();
			jiggleStarted = true;
			System.out.println("Done waiting for shooter");
			
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
        m_shooterSubsystem.resumeShooting();
		m_intakeSubsystem.stopJiggle();
		System.out.println("end of Jiggle command called");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_intakeSubsystem.doneJiggling();
	
	}

}