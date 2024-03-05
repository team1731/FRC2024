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
import frc.robot.subsystems.WristSubsystem;

/**
 * Command to fire into the speaker
 */
public class IntakeCommand extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final IntakeSubsystem m_intakeSubsystem;
    private final WristSubsystem m_wristSubsystem;
	private final ShooterSubsystem m_shooterSubsystem;
	private boolean intakeJiggleStarted = false;




	/**
	 * Creates a new Fire into the speaker
	 *
	 * @param IntakeSubsystem     
	 * @param seqSubsystem        
	 *  
	 */
	public IntakeCommand(IntakeSubsystem intakeSubsystem, WristSubsystem wristSubsystem, ShooterSubsystem shooterSubsystem) {
		m_intakeSubsystem = intakeSubsystem;
		m_wristSubsystem = wristSubsystem;
		m_shooterSubsystem = shooterSubsystem;
		

		// Use addRequirements() here to declare subsystem dependencies.
		if (intakeSubsystem != null && wristSubsystem != null) {
			addRequirements(intakeSubsystem, wristSubsystem, shooterSubsystem);
		}
	}

	// Called when the command is initially scheduled.
	// If it is used as Default command then it gets call all the time
	@Override
	public void initialize() {
		m_wristSubsystem.retractTrapFlap();
		m_wristSubsystem.moveWrist(WristConstants.IntakePosition);
		m_intakeSubsystem.intake(1.0);
		m_intakeSubsystem.initializeJiggle();
		m_shooterSubsystem.reverseSlow();
		intakeJiggleStarted = false;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		//	m_intakeSubsystem.intake(1.0);	
		if (!intakeJiggleStarted && m_intakeSubsystem.hasNote() && m_shooterSubsystem.getShooterVelocity() < -50/60) {
			System.out.println("has the note and shooter reversed");
<<<<<<< HEAD
			intakeJiggleStarted = true; 
			m_intakeSubsystem.feedUpJiggle();
=======
			//intakeJiggleStarted = true; 
			//m_intakeSubsystem.feedUpJiggle();

>>>>>>> origin/integration
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("end of intake called");
        m_intakeSubsystem.stopIntake();
		m_wristSubsystem.moveWrist(WristConstants.wristHomePosition);
		if (m_intakeSubsystem.noteIsPresent()) {
			m_shooterSubsystem.shoot();
		}

		m_intakeSubsystem.stopJiggle();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_intakeSubsystem.doneJiggling();
	
	}

}