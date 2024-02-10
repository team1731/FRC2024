package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends SubsystemBase implements ToggleableSubsystem {

    // motors for the Wrist
    private TalonFX wristMotor1;
    private TalonFX wristMotor2;
    private MotionMagicVoltage mmReq1;
    // private MotionMagicVoltage mmReq2;
     private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

    private boolean enabled;
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public WristSubsystem(boolean enabled) {
        this.enabled = enabled;
        initializeWristMotors();
    }

    /*
     * Wrist MOTOR MOVEMENT
    */

    public double getWristPosition() {
        double position1 = 0;
        double position2 = 0;
        if (enabled){
            position1 = wristMotor1.getRotorPosition().getValueAsDouble();
            position2 = wristMotor2.getRotorPosition().getValueAsDouble();
            System.out.println("WristSubsystem: Position(1:2): " + position1 + ":" + position2);
        }
        return position1;
    }

    public void moveWrist(double position) {
        if (enabled){
            wristMotor1.setControl(mmReq1.withPosition(position));
            wristMotor2.setControl(mmReq1.withPosition(position));
        }
    }
    
    public void wristExtended() {
        System.out.println("X - onTrue - wristExtended method called");
        moveWrist(WristConstants.wristExtendedPosition);
    }

    public void wristHome() {
        System.out.println("X - onFalse - wristHome method called");
        moveWrist(WristConstants.wristHomePosition);
    }

    // Initialize Motors
    private void initializeWristMotors() {
        if (!enabled) {
            return;
        }
        System.out.println("WristSubsystem: Starting UP & Initializing Wrist motors !!!!!!");

        // Initialize Motor 1
        wristMotor1 = new TalonFX(WristConstants.wrist1CanId, "canivore1");
        //mmReq1 = new MotionMagicVoltage(0);

        TalonFXConfiguration configs = new TalonFXConfiguration();

        // Factory Default
        //wristMotor1.getConfigurator().apply(cfg1);
        
    
        // Wrist Motor 1 is CCW+
        configs.MotorOutput.Inverted = WristConstants.wrist1Direction;

        // /* Configure current limits */
        // MotionMagicConfigs mm1 = cfg1.MotionMagic;
        // mm1.MotionMagicCruiseVelocity = WristConstants.MMVel; // 5 rotations per second cruise
        // mm1.MotionMagicAcceleration = WristConstants.MMAcc; // Take approximately 0.5 seconds to reach max vel
        // // Take approximately 0.2 seconds to reach max accel 
        // mm1.MotionMagicJerk = WristConstants.MMJerk;
    
        // // initialze PID controller and encoder objects
        // Slot0Configs slot0_1 = cfg1.Slot0;
        // slot0_1.kP = WristConstants.kP;
        // slot0_1.kI = WristConstants.kI;
        // slot0_1.kD = WristConstants.kD;
        // slot0_1.kV = WristConstants.kV;
        // slot0_1.kS = WristConstants.kS; // Approximately 0.25V to get the mechanism moving
    
        // FeedbackConfigs fdb1 = cfg1.Feedback;
        // fdb1.SensorToMechanismRatio = 1;
        // class member variable


        // robot init, set slot 0 gains
        
        configs.Slot0.kV = 0.12;
        configs.Slot0.kP = 0.11;
        configs.Slot0.kI = 0.48;
        configs.Slot0.kD = 0.01;
    
        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;

        // periodic, run velocity control with slot 0 configs,
        // target velocity of 50 rps
    
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = wristMotor1.getConfigurator().apply(configs);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure Wrist Motor 1. Error: " + status.toString());
        }

        // Initialize Motor 2
        wristMotor2 = new TalonFX(WristConstants.wrist2CanId, "canivore1");
        wristMotor2.setControl(new Follower(wristMotor1.getDeviceID(), false));

        // mmReq2 = new MotionMagicVoltage(0);

        //TalonFXConfiguration configs = new TalonFXConfiguration();

        // Factory Default
        // wristMotor2.getConfigurator().apply(cfg2);
 
        // Wrist Motor 2 is CW+
        // cfg2.MotorOutput.Inverted = WristConstants.wristDirection;

        /* Configure current limits */
        // MotionMagicConfigs mm2 = cfg2.MotionMagic;
        // mm2.MotionMagicCruiseVelocity = WristConstants.MMVel; // 5 rotations per second cruise
        // mm2.MotionMagicAcceleration = WristConstants.MMAcc; // Take approximately 0.5 seconds to reach max vel
        // // Take approximately 0.2 seconds to reach max accel 
        // mm2.MotionMagicJerk = WristConstants.MMJerk;

        // initialze PID controller and encoder objects
        // Slot0Configs slot0_2 = cfg2.Slot0;
        // slot0_2.kP = WristConstants.kP;
        // slot0_2.kI = WristConstants.kI;
        // slot0_2.kD = WristConstants.kD;
        // slot0_2.kV = WristConstants.kV;
        // slot0_2.kS = WristConstants.kS; // Approximately 0.25V to get the mechanism moving
        
        // FeedbackConfigs fdb2 = cfg2.Feedback;
        // fdb2.SensorToMechanismRatio = WristConstants.StM_Ratio;
       // slot0Configs.MotorOutput.Inverted = WristConstants.wrist2Direction;
        StatusCode status2 = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status2 = wristMotor2.getConfigurator().apply(configs);
          if (status2.isOK()) break;
        }
        if (!status2.isOK()) {
          System.out.println("Could not configure Wrist Motor 1. Error: " + status2.toString());
        }

        // // Ensure motor 2 follows motor 1
        // wristMotor2.setControl(new Follower(wristMotor1.getDeviceID(), false));
        wristMotor1.setPosition(0);
        wristMotor2.setPosition(0);
    }

}