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

    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;
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
        System.out.println("ShooterSubsystem: Initializing shooter motors!!!!!!!!!!!!!!!!!!!!!!!!!");
        shooterMotor1 = new CANSparkMax(11, MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(12, MotorType.kBrushless);
        shooterMotor1.restoreFactoryDefaults();
        shooterMotor2.restoreFactoryDefaults();
        shooterMotor1.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
        shooterMotor2.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
        shooterMotor1.setInverted(false);
        shooterMotor2.setInverted(false);
        shooterMotor1.setIdleMode(IdleMode.kBrake);
        shooterMotor2.setIdleMode(IdleMode.kBrake);
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
          double shootSpeed = 1;
          //shootSpeed = -1 * ArmConstants.cubeIntakeSpeed;
          System.out.println("ShooterSubsystem: speed = " + shootSpeed);
          shoot(shootSpeed);
		}

    }

    public void shoot(double shooterSpeed) {
        if (enabled){
          //shooterMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
          shooterMotor1.set(shooterSpeed);
          shooterMotor2.set(shooterSpeed);
          System.out.println(shooterSpeed);
		}
    }

    public void eject() {
        if (enabled){
          //shooterMotor.setSmartCurrentLimit(ArmConstants.EJECT_CURRENT_LIMIT);
          // shooterMotor.set((stateMachine.getGamePiece() == GamePiece.CONE)? -1.0 : 1.0);
          shooterMotor1.set(-1.0);
          shooterMotor2.set(1);
		}
    }

    public void holdShooting() {
        if (enabled){
            shooterMotor1.setSmartCurrentLimit(ArmConstants.INTAKE_HOLD_CURRENT_LIMIT_A);
            // shooterMotor.set((stateMachine.getGamePiece() == GamePiece.CONE)? ArmConstants.INTAKE_HOLD_POWER : -1 * ArmConstants.INTAKE_HOLD_POWER);
            shooterMotor1.set(ArmConstants.INTAKE_HOLD_POWER);
            shooterMotor2.set(ArmConstants.INTAKE_HOLD_POWER);
        }
    }

    public void stopShooting() {
        if (enabled){
        //shooterMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
          shooterMotor1.set(0);
          shooterMotor2.set(0);
        }
    }
    public boolean isIntakeAtStartedVelocity() {
        if (enabled){
            return (Math.abs(shooterMotor1.getEncoder().getVelocity()) > ArmConstants.intakeStartedVelocityThreshold);
        }
        else{
            return false;
        }
    }

    public boolean isIntakeBelowStartedVelocity() {
        if (enabled){
            return (Math.abs(shooterMotor1.getEncoder().getVelocity()) < ArmConstants.intakeStartedVelocityThreshold);
        }
        else{
            return false;
            }
        
    }

    public boolean isIntakeAtHoldingVelocity() {
        if (enabled){
            return (Math.abs(shooterMotor1.getEncoder().getVelocity()) < ArmConstants.intakeHoldingVelocityThreshold);
        }
        else{
            return false;
        }
    }

}

