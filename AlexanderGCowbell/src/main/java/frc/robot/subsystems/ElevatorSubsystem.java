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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase implements ToggleableSubsystem {

    // motors for the Elevator
    private TalonFX elevatorMotor1;
    private TalonFX elevatorMotor2;
    private MotionMagicVoltage mmReq1;
    // private MotionMagicVoltage mmReq2;
     private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);


    private boolean enabled;
    @Override
    public boolean isEnabled() {
        return enabled;
    }

    public ElevatorSubsystem(boolean enabled) {
        this.enabled = enabled;
        initializeElevatorMotors();
    }
    
    /*
     * Elevator MOTOR MOVEMENT
    */

    public double getElevatorPosition() {
        double position1 = 0;
        double position2 = 0;
        if (enabled){
            position1 = elevatorMotor1.getRotorPosition().getValueAsDouble();
            position2 = elevatorMotor2.getRotorPosition().getValueAsDouble();
            System.out.println("ElevatorSubsystem: Position(1:2): " + position1 + ":" + position2);
        }
        //return ((position1 + position2)/2);
        return position1;
    }

    public double getElevator2Position() {
        double position1 = 0;
        double position2 = 0;
        if (enabled){
            position1 = elevatorMotor1.getRotorPosition().getValueAsDouble();
            position2 = elevatorMotor2.getRotorPosition().getValueAsDouble();
            System.out.println("ElevatorSubsystem: Position(1:2): " + position1 + ":" + position2);
        }
        //return ((position1 + position2)/2);
        return position2;
    }

    private void moveElevator(double position) {
        if (enabled){
            elevatorMotor1.setControl(mmReq1.withPosition(position));
            elevatorMotor2.setControl(mmReq1.withPosition(position));
        }
    }
    
    public void sendElevatorUp() {
        System.out.println("X - onTrue - sendElevatorUp method called");
        moveElevator(ElevatorConstants.elevatorExtendedPosition);
	
    }

    public void sendElevatorHome() {
        System.out.println("X - onFalse - sendElevatorHome method called");
        moveElevator(ElevatorConstants.elevatorHomePosition);
    }
    // Initialize Motors
    private void initializeElevatorMotors() {
        if (!enabled) {
            return;
        }
        System.out.println("ElevatorSubsystem: Starting UP & Initializing Elevator motors !!!!!!");

        // Initialize Motor 1
        elevatorMotor1 = new TalonFX(ElevatorConstants.elevatorCancoderId1, "canivore1");
       // mmReq1 = new MotionMagicVoltage(0);

       // TalonFXConfiguration cfg1 = new TalonFXConfiguration();
       TalonFXConfiguration config = new TalonFXConfiguration();

        // Factory Default
        //elevatorMotor1.getConfigurator().apply(cfg1);
        
    
        // Elevator Motor 1 is CCW+
        //cfg1.MotorOutput.Inverted = ElevatorConstants.elevatorDirection;

        // /* Configure current limits */
        // MotionMagicConfigs mm1 = cfg1.MotionMagic;
        // mm1.MotionMagicCruiseVelocity = ElevatorConstants.MMVel; // 5 rotations per second cruise
        // mm1.MotionMagicAcceleration = ElevatorConstants.MMAcc; // Take approximately 0.5 seconds to reach max vel
        // // Take approximately 0.2 seconds to reach max accel 
        // mm1.MotionMagicJerk = ElevatorConstants.MMJerk;
    
        // initialze PID controller and encoder objects
       //Slot0Configs slot0_1 = cfg1.Slot0;
        Slot0Configs slot0_1 = config.Slot0;
        slot0_1.kP = ElevatorConstants.kP;
        slot0_1.kI = ElevatorConstants.kI;
        slot0_1.kD = ElevatorConstants.kD;
        slot0_1.kV = ElevatorConstants.kV;
        slot0_1.kS = ElevatorConstants.kS; // Approximately 0.25V to get the mechanism moving


        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;
    
        //FeedbackConfigs fdb1 = cfg1.Feedback;
        FeedbackConfigs fdb1 = config.Feedback;
        fdb1.SensorToMechanismRatio = ElevatorConstants.StM_Ratio;
    
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status = elevatorMotor1.getConfigurator().apply(config);
          if (status.isOK()) break;
        }
        if (!status.isOK()) {
          System.out.println("Could not configure Elevator Motor 1. Error: " + status.toString());
        }

        // Initialize Motor 2
        elevatorMotor2 = new TalonFX(ElevatorConstants.elevatorCancoderId2, "canivore1");
        elevatorMotor2.setControl(new Follower(elevatorMotor1.getDeviceID(), true));

        // mmReq2 = new MotionMagicVoltage(0);

        // TalonFXConfiguration cfg2 = new TalonFXConfiguration();

        // Factory Default
        // elevatorMotor2.getConfigurator().apply(cfg2);
 
        // Elevator Motor 2 is CW+
        // cfg2.MotorOutput.Inverted = ElevatorConstants.elevatorDirection;

        /* Configure current limits */
        // MotionMagicConfigs mm2 = cfg2.MotionMagic;
        // mm2.MotionMagicCruiseVelocity = ElevatorConstants.MMVel; // 5 rotations per second cruise
        // mm2.MotionMagicAcceleration = ElevatorConstants.MMAcc; // Take approximately 0.5 seconds to reach max vel
        // // Take approximately 0.2 seconds to reach max accel 
        // mm2.MotionMagicJerk = ElevatorConstants.MMJerk;

        // initialze PID controller and encoder objects
        // Slot0Configs slot0_2 = cfg2.Slot0;
        // slot0_2.kP = ElevatorConstants.kP;
        // slot0_2.kI = ElevatorConstants.kI;
        // slot0_2.kD = ElevatorConstants.kD;
        //slot0_2.kV = ElevatorConstants.kV;
        // slot0_2.kS = ElevatorConstants.kS; // Approximately 0.25V to get the mechanism moving
        
        // FeedbackConfigs fdb2 = cfg2.Feedback;
        // fdb2.SensorToMechanismRatio = ElevatorConstants.StM_Ratio;
 
        StatusCode status2 = StatusCode.StatusCodeNotInitialized;
        for(int i = 0; i < 5; ++i) {
          status2 = elevatorMotor2.getConfigurator().apply(config);
          if (status2.isOK()) break;
        }
        if (!status2.isOK()) {
          System.out.println("Could not configure Elevator Motor 1. Error: " + status2.toString());
        }

        // // Ensure motor 2 follows motor 1
        // elevatorMotor2.setControl(new Follower(elevatorMotor1.getDeviceID(), false));
        elevatorMotor1.setPosition(0);
        elevatorMotor2.setPosition(0);

        
    }
    public void periodic() {
        SmartDashboard.putNumber("Elevayor motor 1 position", getElevatorPosition());
        SmartDashboard.putNumber("Elevator motor 2 position", getElevator2Position());

    }
    public void liftElevator(){
        elevatorMotor1.setControl(m_voltageVelocity.withVelocity(-10));
    }
    public void lowerElevator(){
        elevatorMotor1.setControl(m_voltageVelocity.withVelocity(10));
    }
}
