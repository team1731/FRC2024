/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command to fire into the speaker
 */
public class AutoSetWrist extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final IntakeSubsystem m_intakeSubsystem;
    private final WristSubsystem m_wristSubsystem;
	private double m_wristHeight;


	/**
	 * Creates a new Fire into the speaker
	 *
	 * @param IntakeSubsystem     
	 * @param seqSubsystem        
	 *  
	 */
	public AutoSetWrist(double height, IntakeSubsystem intakeSubsystem, WristSubsystem wristSubsystem) {
		m_intakeSubsystem = intakeSubsystem;
		m_wristSubsystem = wristSubsystem;
		m_wristHeight = height;
		

		// Use addRequirements() here to declare subsystem dependencies.
		if (intakeSubsystem != null && wristSubsystem != null) {
			addRequirements(intakeSubsystem, wristSubsystem);
		}
	}

	// Called when the command is initially scheduled.
	// If it is used as Default command then it gets call all the time
	@Override
	public void initialize() {
		

		m_wristSubsystem.moveWrist(m_wristHeight);
		
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
				
	
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
      //  m_intakeSubsystem.stopIntake();
	//	m_wristSubsystem.moveWrist(WristConstants.wristHomePosition);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}

}