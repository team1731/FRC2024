package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Robot;
import frc.robot.Constants.Swerve.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase implements ToggleableSubsystem{

    private CANSparkMax shooterMotor1;
    private CANSparkMax shooterMotor2;
    private SparkPIDController shooterPIDController1;
    private SparkPIDController shooterPIDController2;
    private boolean enabled;
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public ShooterSubsystem(boolean enabled) {
        this.enabled = enabled;
        initializeShooterMotor();
    }

    private void initializeShooterMotor() {
        if (enabled) {
            System.out.println("ShooterSubsystem: Starting Up & Initializing shooter motors !!!!");
            shooterMotor1 = new CANSparkMax(11, MotorType.kBrushless);
            shooterMotor2 = new CANSparkMax(12, MotorType.kBrushless);
            shooterMotor1.restoreFactoryDefaults();
            shooterMotor2.restoreFactoryDefaults();
            shooterMotor1.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT_A);
            shooterMotor2.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT_A);
            shooterMotor1.setInverted(false);
            shooterMotor2.setInverted(false);
            shooterMotor1.setIdleMode(IdleMode.kCoast);
            shooterMotor2.setIdleMode(IdleMode.kCoast);

            // initialze PID controller and encoder objects
            shooterPIDController1 = shooterMotor1.getPIDController();
            shooterPIDController2 = shooterMotor2.getPIDController();

            // set PID coefficients
            shooterPIDController1.setP(ShooterConstants.kP);
            shooterPIDController1.setI(ShooterConstants.kI);
            shooterPIDController1.setD(ShooterConstants.kD);
            shooterPIDController1.setIZone(ShooterConstants.kIz);
            shooterPIDController1.setFF(ShooterConstants.kFF);
            shooterPIDController1.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);   

            shooterPIDController2.setP(ShooterConstants.kP);
            shooterPIDController2.setI(ShooterConstants.kI);
            shooterPIDController2.setD(ShooterConstants.kD);
            shooterPIDController2.setIZone(ShooterConstants.kIz);
            shooterPIDController2.setFF(ShooterConstants.kFF);
            shooterPIDController2.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);   
		}
    }

    public void shoot(){
        if (enabled){
            if (Robot.doSD()) { 
                System.out.println("ShooterSubsystem: m1speed, m2speed = " + ShooterConstants.kMotorSpeed1 + ", " + ShooterConstants.kMotorSpeed2); 
            }
            shooterMotor1.set(ShooterConstants.kMotorSpeed1);
            shooterMotor2.set(ShooterConstants.kMotorSpeed2);
		}  

    }    

    public void stopShooting() {
        if (enabled){
          shooterMotor1.set(0);
          shooterMotor2.set(0);
        }
    }

    /*

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
    */
}

