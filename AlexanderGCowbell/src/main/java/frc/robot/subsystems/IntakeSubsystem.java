package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GamePiece;
import frc.robot.state.arm.ArmStateMachine.MovementType;

public class IntakeSubsystem  extends SubsystemBase {
    //private CANSparkMax intakeMotor;
    
    public IntakeSubsystem() {
        System.out.println("IntakeSubsystem: Starting up!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        initializeIntakeMotor();
    }

    private void initializeIntakeMotor() {
        System.out.println("IntakeSubsystem: Initializing arm motors!!!!!!!!!!!!!!!!!!!!!!!!!");
       // intakeMotor = new CANSparkMax(ArmConstants.intakeCancoderId, MotorType.kBrushless);
        // intakeMotor.restoreFactoryDefaults();
        // intakeMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
        // intakeMotor.setInverted(false);
        // intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    /*
     * INTAKE MOTOR MOVEMENT
     */

    public void intake() {
        double intakeSpeed = 0.75;
        // if(stateMachine.getMovementType() == MovementType.PICKUP_DOWNED_CONE) {
        //     intakeSpeed = ArmConstants.downedConeIntakeSpeed;
        //     System.out.println("IntakeSubsystem: intaking DOWNED CONE, speed = " + intakeSpeed);
        // } else if(stateMachine.getGamePiece() == GamePiece.CONE) {
        //     intakeSpeed =  ArmConstants.coneIntakeSpeed;
        //     System.out.println("IntakeSubsystem: intaking CONE, speed = " + intakeSpeed);
        // } else {
        //     intakeSpeed = ArmConstants.cubeIntakeSpeed;
        //     System.out.println("IntakeSubsystem: intaking CUBE, speed = " + intakeSpeed);
        // }
        intakeSpeed = ArmConstants.cubeIntakeSpeed;
        System.out.println("IntakeSubsystem: speed = " + intakeSpeed);
        intake(intakeSpeed);
    }

    public void reverseIntake() {
        double intakeSpeed = 0.75;
        intakeSpeed = -1 * ArmConstants.cubeIntakeSpeed;
        System.out.println("IntakeSubsystem: speed = " + intakeSpeed);
        intake(intakeSpeed);
    }

    public void intake(double intakeSpeed) {
        // intakeMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
        // intakeMotor.set(intakeSpeed);
    }

    public void eject() {
        // intakeMotor.setSmartCurrentLimit(ArmConstants.EJECT_CURRENT_LIMIT);
        // // intakeMotor.set((stateMachine.getGamePiece() == GamePiece.CONE)? -1.0 : 1.0);
        // intakeMotor.set(-1.0);
    }

    public void holdIntake() {
        // intakeMotor.setSmartCurrentLimit(ArmConstants.INTAKE_HOLD_CURRENT_LIMIT_A);
        // // intakeMotor.set((stateMachine.getGamePiece() == GamePiece.CONE)? ArmConstants.INTAKE_HOLD_POWER : -1 * ArmConstants.INTAKE_HOLD_POWER);
        // intakeMotor.set(ArmConstants.INTAKE_HOLD_POWER);
    }

    public void stopIntake() {
        // intakeMotor.setSmartCurrentLimit(ArmConstants.INTAKE_CURRENT_LIMIT_A);
        // intakeMotor.set(0);
    }

    public boolean isIntakeAtStartedVelocity() {
        //return (Math.abs(intakeMotor.getEncoder().getVelocity()) > ArmConstants.intakeStartedVelocityThreshold);
        return(false);
    }

    public boolean isIntakeBelowStartedVelocity() {
        //return (Math.abs(intakeMotor.getEncoder().getVelocity()) < ArmConstants.intakeStartedVelocityThreshold);
        return(false);
    }

    public boolean isIntakeAtHoldingVelocity() {
        //return (Math.abs(intakeMotor.getEncoder().getVelocity()) < ArmConstants.intakeHoldingVelocityThreshold);
        return(false);
    }

}
