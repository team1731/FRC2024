package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.state.arm.ArmStateMachine;

public class IntakeSubsystem  extends SubsystemBase {
    private CANSparkMax intakeMotor;
    
    public IntakeSubsystem() {
        System.out.println("IntakeSubsystem: Starting up!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }

    private void initializeIntakeMotor() {
        System.out.println("IntakeSubsystem: Initializing arm motors!!!!!!!!!!!!!!!!!!!!!!!!!");
        intakeMotor = new CANSparkMax(ArmConstants.intakeCancoderId, CANSparkLowLevel.MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
        intakeMotor.setInverted(false);
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }
}
