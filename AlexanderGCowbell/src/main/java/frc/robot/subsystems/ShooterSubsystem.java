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

public class ShooterSubsystem extends SubsystemBase implements ToggleableSubsystem{

    private CANSparkMax shooterMotor;
    private boolean enabled;
    @Override
    public boolean isEnabled() {
        return enabled;
    }
    public ShooterSubsystem(boolean enabled) {
        this.enabled = enabled;
        if (enabled){
            System.out.println("ShooterSubsystem: Starting up!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            initializeShooterMotor();
        }
    }
    private void initializeShooterMotor() {
        if (enabled){
            System.out.println("ShooterSubsystem: Initializing arm motors!!!!!!!!!!!!!!!!!!!!!!!!!");
            shooterMotor = new CANSparkMax(1, MotorType.kBrushless);
            shooterMotor.restoreFactoryDefaults();
            shooterMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
            shooterMotor.setInverted(false);
            shooterMotor.setIdleMode(IdleMode.kBrake);
        }
    }

    // public void reverseIntake() {
    //     double intakeSpeed = 0.75;
    //     intakeSpeed = -1 * ArmConstants.cubeIntakeSpeed;
    //     System.out.println("IntakeSubsystem: speed = " + intakeSpeed);
    //     shoot(intakeSpeed);
    // }


    public void shoot(){
        if (enabled){
            double shootSpeed = 0.75;
            shootSpeed = -1 * ArmConstants.cubeIntakeSpeed;
            System.out.println("ShooterSubsystem: speed = " + shootSpeed);
            shoot(shootSpeed);
        }
    }

    public void shoot(double shooterSpeed) {
        if (enabled){
            shooterMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
            shooterMotor.set(shooterSpeed);
        }
    }

    public void eject() {
        if (enabled){
            shooterMotor.setSmartCurrentLimit(ArmConstants.EJECT_CURRENT_LIMIT);
            // shooterMotor.set((stateMachine.getGamePiece() == GamePiece.CONE)? -1.0 : 1.0);
            shooterMotor.set(-1.0);
        }
    }

    public void holdShooting() {
        if (enabled){
            shooterMotor.setSmartCurrentLimit(ArmConstants.INTAKE_HOLD_CURRENT_LIMIT_A);
            // shooterMotor.set((stateMachine.getGamePiece() == GamePiece.CONE)? ArmConstants.INTAKE_HOLD_POWER : -1 * ArmConstants.INTAKE_HOLD_POWER);
            shooterMotor.set(ArmConstants.INTAKE_HOLD_POWER);
        }
    }

    public void stopShooting() {
        if (enabled){
            shooterMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
            shooterMotor.set(0);
        }
    }
    public boolean isIntakeAtStartedVelocity() {
        if (enabled){
            return (Math.abs(shooterMotor.getEncoder().getVelocity()) > ArmConstants.intakeStartedVelocityThreshold);
        }
        else{
            return false;
        }
    }

    public boolean isIntakeBelowStartedVelocity() {
        if (enabled){
            return (Math.abs(shooterMotor.getEncoder().getVelocity()) < ArmConstants.intakeStartedVelocityThreshold);
        }
        else{
            return false;
            }
        
    }

    public boolean isIntakeAtHoldingVelocity() {
        if (enabled){
            return (Math.abs(shooterMotor.getEncoder().getVelocity()) < ArmConstants.intakeHoldingVelocityThreshold);
        }
        else{
            return false;
        }
    }

}

