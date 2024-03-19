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
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command to fire into the speaker
 */
public class AutoUseVision extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final IntakeSubsystem m_intakeSubsystem;
    private final WristSubsystem m_wristSubsystem;
	private final VisionSubsystem m_visionSubsystem;
	private double m_wristHeight;


	/**
	 * Creates a new Fire into the speaker
	 *
	 * @param IntakeSubsystem     
	 * @param seqSubsystem        
	 *  
	 */
	public AutoUseVision(IntakeSubsystem intakeSubsystem, WristSubsystem wristSubsystem, VisionSubsystem visionSubsystem) {
		m_intakeSubsystem = intakeSubsystem;
		m_wristSubsystem = wristSubsystem;
		m_visionSubsystem = visionSubsystem;

		// Use addRequirements() here to declare subsystem dependencies.
		if (intakeSubsystem != null && wristSubsystem != null) {
			addRequirements(intakeSubsystem, wristSubsystem, visionSubsystem);
		}
	}

	// Called when the command is initially scheduled.
	// If it is used as Default command then it gets call all the time
	@Override
	public void initialize() {
		m_wristSubsystem.startMoveWristToTarget();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_wristSubsystem.setWristBasedOnDistance(m_visionSubsystem.getDistanceToTargetForAuto());
	
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
    
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return !m_wristSubsystem.getAutoVisionFlag();
	}

}