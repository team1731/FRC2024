package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.shooterConstants;
import frc.robot.state.arm.ArmStateMachine.MovementType;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.state.arm.ArmStateMachine.MovementType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{

    private CANSparkMax shooterMotor;
    private CANSparkMax shooterMotor2;

    public ShooterSubsystem() {
        System.out.println("ShooterSubsystem: Starting up!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        initializeShooterMotor();
    }
    private void initializeShooterMotor() {
        System.out.println("ShooterSubsystem: Initializing arm motors!!!!!!!!!!!!!!!!!!!!!!!!!");
        shooterMotor = new CANSparkMax(1, MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(2, MotorType.kBrushless);
        shooterMotor.restoreFactoryDefaults();
        shooterMotor2.restoreFactoryDefaults();
        shooterMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
        shooterMotor2.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
        shooterMotor.setInverted(false);
        shooterMotor2.setInverted(false);
        shooterMotor.setIdleMode(IdleMode.kBrake);
        shooterMotor2.setIdleMode(IdleMode.kBrake);
    }

    // public void reverseIntake() {
    //     double intakeSpeed = 0.75;
    //     intakeSpeed = -1 * ArmConstants.cubeIntakeSpeed;
    //     System.out.println("IntakeSubsystem: speed = " + intakeSpeed);
    //     shoot(intakeSpeed);
    // }


    public void shoot(){
        double shootSpeed = 1;
        //shootSpeed = -1 * ArmConstants.cubeIntakeSpeed;
        System.out.println("ShooterSubsystem: speed = " + shootSpeed);
        shoot(shootSpeed);

    }

    public void shoot(double shooterSpeed) {
        //shooterMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
        shooterMotor.set(shooterSpeed);
        shooterMotor2.set(-shooterSpeed);
        System.out.println(-shooterSpeed);
    }

    public void eject() {
        //shooterMotor.setSmartCurrentLimit(ArmConstants.EJECT_CURRENT_LIMIT);
        // shooterMotor.set((stateMachine.getGamePiece() == GamePiece.CONE)? -1.0 : 1.0);
        shooterMotor.set(-1.0);
        shooterMotor2.set(1);
    }

    public void holdShooting() {
        shooterMotor.setSmartCurrentLimit(ArmConstants.INTAKE_HOLD_CURRENT_LIMIT_A);
        // shooterMotor.set((stateMachine.getGamePiece() == GamePiece.CONE)? ArmConstants.INTAKE_HOLD_POWER : -1 * ArmConstants.INTAKE_HOLD_POWER);
        shooterMotor.set(ArmConstants.INTAKE_HOLD_POWER);
    }

    public void stopShooting() {
        //shooterMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
        shooterMotor.set(0);
        shooterMotor2.set(0);
    }
    public boolean isIntakeAtStartedVelocity() {
        return (Math.abs(shooterMotor.getEncoder().getVelocity()) > ArmConstants.intakeStartedVelocityThreshold);
    }

    public boolean isIntakeBelowStartedVelocity() {
        return (Math.abs(shooterMotor.getEncoder().getVelocity()) < ArmConstants.intakeStartedVelocityThreshold);
    }

    public boolean isIntakeAtHoldingVelocity() {
        return (Math.abs(shooterMotor.getEncoder().getVelocity()) < ArmConstants.intakeHoldingVelocityThreshold);
    }

}

