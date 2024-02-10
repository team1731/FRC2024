package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.DemandType;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.sensors.CANCoder;

//import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix.motorcontrol.DemandType;
//import com.ctre.phoenix.motorcontrol.DemandType;
//import com.ctre.phoenix.motorcontrol.DemandType;
//import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;
    private double lastAngle;
    //private int falcontics = 0;
    //private DebugValues debugValues;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);
    VoltageOut m_request = new VoltageOut(0);
    VelocityVoltage m_velocity = new VelocityVoltage(0);
    TalonFXConfiguration m_TFX = new TalonFXConfiguration();
    PositionVoltage m_positionVoltage = new PositionVoltage(0);
    CANcoderConfiguration m_canConfig = new CANcoderConfiguration();
    TalonFXConfiguration m_talConfig = new TalonFXConfiguration();

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        //debugValues = new DebugValues(moduleNumber);
        angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID,"canivore1");
        configAngleEncoder();
    

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID,"canivore1");
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID,"canivore1");
        configDriveMotor();

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean lockAngle){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        if(isOpenLoop){
            double outputVoltage = 12 * desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            //outputVoltage is multiplied by 12 because 12 volts is full speed
            mDriveMotor.setControl(m_request.withOutput(outputVoltage));
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.setControl(m_velocity.withVelocity(velocity));
        }
//convert form 
        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) && !lockAngle ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.

        mAngleMotor.setControl(m_velocity.withVelocity(Conversions.degreesToFalcon(angle, Constants.Swerve.angleGearRatio))); 
        lastAngle = angle;
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, Constants.Swerve.angleGearRatio);
        System.err.println("Cancoder degrees" + getCanCoder().getDegrees());
        System.err.println("angleoffset" + angleOffset);
        System.err.println("absolutePosition" + absolutePosition);
        mAngleMotor.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.getConfigurator().apply(m_canConfig);
       // angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.getConfigurator().apply(m_talConfig);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
      //  mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.getConfigurator().apply(m_talConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setPosition(0);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue());
    }

    public SwerveModuleState getState(){
        double velocity = Conversions.falconToMPS(mDriveMotor.getRotorVelocity().getValue(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getRotorPosition().getValue(), Constants.Swerve.angleGearRatio));
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition(){
        double position = ((mDriveMotor.getRotorPosition().getValue()/2048.0) * Constants.Swerve.wheelCircumference) / Constants.Swerve.driveGearRatio;
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getRotorPosition().getValue(), Constants.Swerve.angleGearRatio));
        return new SwerveModulePosition(position, angle);
    }
    
    public class DebugValues {
		public int id;

		public double drive;
		public double turningMotorOutput;
		public double turnAppliedOutput;
		public double turnVelocity;
		public double driveAppliedOutput;
		public double driveVelocity;

		public DebugValues(int id) {
			this.id = id;
		}

		public void update(double drive, double turningMotorOutput, double turnAppliedOutput, double turnVelocity,
				double driveAppliedOutput, double driveVelocity) {
			this.drive = drive;
			this.turningMotorOutput = turningMotorOutput;
			this.turnAppliedOutput = turnAppliedOutput;
			this.turnVelocity = turnVelocity;
			this.driveAppliedOutput = driveAppliedOutput;
			this.driveVelocity = driveVelocity;
		}
	}
}

//todo: give values to m_talConfig and m_canConfig