package frc.robot.subsystems;


import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.util.LinearServo;

public class WristSubsystem extends SubsystemBase implements ToggleableSubsystem {

    // motors for the Wrist
    private TalonFX wristMotor1;
    private TalonFX wristMotor2;
    private MotionMagicVoltage mmReq1 = new MotionMagicVoltage(0);
    private Servo trapFlapServo = new LinearServo(0, 50,20 );

    // private MotionMagicVoltage mmReq2;
    // private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

    private boolean enabled;

    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public WristSubsystem(boolean enabled) {
        this.enabled = enabled;
        if(enabled){
            initializeWristMotors();
        }
    }

    /*
     * Wrist MOTOR MOVEMENT
    */

    public void moveWrist(double position) {
        if(!enabled) return;
        wristMotor1.setControl(mmReq1.withPosition(position));
        wristMotor2.setControl(mmReq1.withPosition(position));
        System.out.println("MOving wrist to" + position);
    }


    // Initialize Motors
    private void initializeWristMotors() {
        if (!enabled) {
            return;
        }
        System.out.println("WristSubsystem: Starting UP & Initializing Wrist motors !!!!!!");

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        //Set Defaults
        
        wristMotor1 = new TalonFX(WristConstants.wrist1CanId, "canivore1");
        wristMotor2 = new TalonFX(WristConstants.wrist2CanId, "canivore1");
        wristMotor1.getConfigurator().apply(cfg);
        wristMotor2.getConfigurator().apply(cfg);
        
        wristMotor1.setNeutralMode(NeutralModeValue.Brake);
        wristMotor2.setNeutralMode(NeutralModeValue.Brake);

        /* Configure current limits */
        MotionMagicConfigs mm = cfg.MotionMagic;
        mm.MotionMagicCruiseVelocity = 80; // 5 rotations per second cruise
        mm.MotionMagicAcceleration = 300; // Take approximately 0.5 seconds to reach max vel
        // Take approximately 0.2 seconds to reach max accel 
        mm.MotionMagicJerk = 0;
        

        Slot0Configs slot0 = cfg.Slot0;
        slot0.kP = 4.6875;
        slot0.kI = 0;
        slot0.kD = 0.0078125;
        slot0.kV = 0.009375;
        slot0.kS = 0.01953125; // Approximately 0.25V to get the mechanism moving

        FeedbackConfigs fdb = cfg.Feedback;
        fdb.SensorToMechanismRatio = 1;




        // Apply the configs for Motor 1
        cfg.MotorOutput.Inverted = WristConstants.wrist2Direction;
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
        status = wristMotor1.getConfigurator().apply(cfg);
        if (status.isOK()) break;
        }
        if (!status.isOK()) {
        System.out.println("Could not configure device. Error: " + status.toString());
        }

        // Apply the configs for Motor 2
        cfg.MotorOutput.Inverted = WristConstants.wrist1Direction;
        status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
        status = wristMotor2.getConfigurator().apply(cfg);
        if (status.isOK()) break;
        }
        if (!status.isOK()) {
        System.out.println("Could not configure device. Error: " + status.toString());
        }
        
        wristMotor1.setPosition(0);
        wristMotor2.setPosition(0);

    }
    public void periodic() {
        if(!enabled) return;
        SmartDashboard.putNumber("Wrist motor 1 position", wristMotor1.getRotorPosition().getValueAsDouble());
        SmartDashboard.putNumber("Wrist motor 2 position", wristMotor2.getRotorPosition().getValueAsDouble());
    }

    public double getWristPosition() {
        if(!enabled) return 0;
        return wristMotor1.getPosition().getValueAsDouble();
    }

    public void extendTrapFlap() {
        trapFlapServo.setPosition(0);
        System.out.println("extending");
      
       
    }

    public void retractTrapFlap() {
      trapFlapServo.setPosition(45);
      System.out.println("retracting");
    }

}
