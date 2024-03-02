package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;

public class ShooterSubsystem extends SubsystemBase implements ToggleableSubsystem{

  //  private CANSparkMax shooterMotor1;
 //   private CANSparkMax shooterMotor2;
  //  private SparkPIDController shooterPIDController1;
 //   private SparkPIDController shooterPIDController2;
    private static final String canBusName = Constants.CANBUS_NAME;
    private final TalonFX m_fx = new TalonFX(Constants.ShooterConstants.shooterCancoderId1, canBusName);
    private final TalonFX m_fllr = new TalonFX(Constants.ShooterConstants.shooterCancoderId2, canBusName);
    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    private final NeutralOut m_brake = new NeutralOut();
    private boolean enabled;
    private boolean isShooting = false;
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
            TalonFXConfiguration configs = new TalonFXConfiguration();

            /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
            configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
            configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
            configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
            configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
            // Peak output of 8 volts
            configs.Voltage.PeakForwardVoltage = 12;
            configs.Voltage.PeakReverseVoltage = -12;

                /* Retry config apply up to 5 times, report if failure */
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5; ++i) {
            status = m_fx.getConfigurator().apply(configs);
            if (status.isOK()) break;
            }
            if(!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
            }

            m_fllr.setControl(new Follower(m_fx.getDeviceID(), false));
            }
                    
        }

    public void shoot(){
        if (enabled){
            if (Robot.doSD()) { 
                System.out.println("ShooterSubsystem: m1speed, m2speed = " + ShooterConstants.kMotorSpeed1 + ", " + ShooterConstants.kMotorSpeed2); 
            }
            isShooting = true;
            m_fx.setControl(m_voltageVelocity.withVelocity(5000.0/60));
		}  

    }    

    public void stopShooting() {
        if (enabled){
            isShooting = false;
            m_fx.setControl(m_brake);
        }
    }

    public void resumeShooting() {
        if (isShooting) {
            shoot();
        } else {
            stopShooting();
        }
    }

    public void pauseShooting() {
        if (enabled){
            m_fx.setControl(m_brake);
        }
    }

    public void shootAmp() {
            m_fx.setControl(m_voltageVelocity.withVelocity(1000.0/60));
    }

    public void reverseSlow() {

            m_fx.setControl(m_voltageVelocity.withVelocity(-500.0/60));
    }

    public double getShooterVelocity() {
        return m_fx.getVelocity().getValueAsDouble();
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

