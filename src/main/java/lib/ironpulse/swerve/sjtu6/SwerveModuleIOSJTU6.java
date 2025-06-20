package lib.ironpulse.swerve.sjtu6;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.SwerveModuleParamsNT;
import lib.ironpulse.swerve.SwerveModuleIO;
import lib.ironpulse.swerve.SwerveConfig;

import static edu.wpi.first.units.Units.*;

public class SwerveModuleIOSJTU6 implements SwerveModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final SwerveSJTU6Config config;
    
    // Control requests
    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0);
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0);
    private final VoltageOut steerVoltageRequest = new VoltageOut(0);
    
    // Configuration objects - separate FB and FF to avoid overwriting settings
    private final TalonFXConfiguration driveFBConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration driveFFConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration steerFBConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration driveBrakeConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration steerBrakeConfig = new TalonFXConfiguration();
    
    //FIXME: config
    public SwerveModuleIOSJTU6(SwerveSJTU6Config config, int driveMotorId, int steerMotorId, boolean driveInverted, boolean steerInverted) {
        this.config = config;
        
        // Initialize motors 
        driveMotor = new TalonFX(driveMotorId);
        steerMotor = new TalonFX(steerMotorId);
        
        // Configure motors
        configureDriveMotor(driveInverted);
        configureSteerMotor(steerInverted);
        
        // Optimize bus utilization
        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
    }
    
    private void configureDriveMotor(boolean inverted) {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        
        // Motor output
        driveConfig.MotorOutput.Inverted = inverted ? 
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Current limits - reasonable defaults for SJTU6
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = config.slipCurrent.in(Amp);
        
        // PID configuration - use defaults, will be updated by periodic calls
        driveConfig.Slot0.kP = SwerveModuleParamsNT.driveKp();
        driveConfig.Slot0.kI = SwerveModuleParamsNT.driveKi();
        driveConfig.Slot0.kD = SwerveModuleParamsNT.driveKd();
        driveConfig.Slot0.kS = SwerveModuleParamsNT.driveKs();
        driveConfig.Slot0.kV = SwerveModuleParamsNT.driveKv();
        driveConfig.Slot0.kA = SwerveModuleParamsNT.driveKa();
        
        // Apply configuration
        driveMotor.getConfigurator().apply(driveConfig);
    }
    
    private void configureSteerMotor(boolean inverted) {
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        
        // Motor output
        steerConfig.MotorOutput.Inverted = inverted ? 
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //FIXME: config
        steerConfig.Feedback.FeedbackRemoteSensorID = 0;
        steerConfig.Feedback.RotorToSensorRatio = config.steerGearRatio;
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        
        // Current limits - reasonable defaults for SJTU6
        //TODO: add this to config?
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        
        // PID configuration - use defaults, will be updated by periodic calls
        steerConfig.Slot0.kP = SwerveModuleParamsNT.steerKp();
        steerConfig.Slot0.kI = SwerveModuleParamsNT.steerKi();
        steerConfig.Slot0.kD = SwerveModuleParamsNT.steerKd();
        
        // Continuous wrap for steering
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        // Apply configuration
        steerMotor.getConfigurator().apply(steerConfig);
    }
    
    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // Drive motor inputs
        inputs.driveMotorConnected = driveMotor.getPosition().getStatus().isOK();
        inputs.driveMotorPositionRad = driveMotorRotationsToMechanismRad(driveMotor.getPosition().getValueAsDouble());
        inputs.driveMotorVelocityRadPerSec = driveMotorRotationsToMechanismRad(driveMotor.getVelocity().getValueAsDouble());
        inputs.driveMotorVoltageVolt = driveMotor.getMotorVoltage().getValueAsDouble();
        inputs.driveMotorSupplyCurrentAmpere = driveMotor.getSupplyCurrent().getValueAsDouble();
        inputs.driveMotorTorqueCurrentAmpere = driveMotor.getTorqueCurrent().getValueAsDouble();
        inputs.driveMotorTemperatureCel = driveMotor.getDeviceTemp().getValueAsDouble();
        
        // Drive position samples
        inputs.driveMotorPositionRadSamples = new double[] { inputs.driveMotorPositionRad };
        
        // Steer motor inputs  
        inputs.steerMotorConnected = steerMotor.getPosition().getStatus().isOK();
        inputs.steerMotorPositionRad = steerMotorRotationsToMechanismRad(steerMotor.getPosition().getValueAsDouble());
        inputs.steerMotorVelocityRadPerSec = steerMotorRotationsToMechanismRad(steerMotor.getVelocity().getValueAsDouble());
        inputs.steerMotorVoltageVolt = steerMotor.getMotorVoltage().getValueAsDouble();
        inputs.steerMotorSupplyCurrentAmpere = steerMotor.getSupplyCurrent().getValueAsDouble();
        inputs.steerMotorTorqueCurrentAmpere = steerMotor.getTorqueCurrent().getValueAsDouble();
        inputs.steerMotorTemperatureCel = steerMotor.getDeviceTemp().getValueAsDouble();
        
        // Steer position samples
        inputs.steerMotorPositionRadSamples = new double[] { inputs.steerMotorPositionRad };
    }
    
    @Override
    public void setSwerveModuleState(SwerveModuleState state) {
        // Set drive velocity
        double velocityRps = linearVelocityToDriveMotorRPS(state.speedMetersPerSecond);
        driveMotor.setControl(driveVelocityRequest.withVelocity(velocityRps));
        
        // Set steer angle
        double positionRotations = mechanismRadToSteerMotorRotations(state.angle.getRadians());
        steerMotor.setControl(steerPositionRequest.withPosition(positionRotations));
    }
    
    @Override
    public void setDriveOpenLoop(Voltage voltage) {
        driveMotor.setControl(driveVoltageRequest.withOutput(voltage.in(Volt)));
    }
    
    @Override
    public void setDriveVelocity(LinearVelocity velocity) {
        double velocityRps = linearVelocityToDriveMotorRPS(velocity.in(MetersPerSecond));
        driveMotor.setControl(driveVelocityRequest.withVelocity(velocityRps).withEnableFOC(true));
    }

    //FIXME: what is this?
    @Override
    public void setDriveTorqueFeedforward(Torque torque) {
        // For SJTU6, convert torque to approximate voltage
        double voltage = torque.in(NewtonMeters) * 0.1; // Rough approximation
        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }
    
    @Override
    public void setSteerOpenLoop(Voltage voltage) {
        steerMotor.setControl(steerVoltageRequest.withOutput(voltage.in(Volt)));
    }
    
    @Override
    public void setSteerAngleAbsolute(Angle angle) {
        double positionRotations = mechanismRadToSteerMotorRotations(angle.in(Radian));
        steerMotor.setControl(steerPositionRequest.withPosition(positionRotations).withEnableFOC(true));
    }
    
    @Override
    public void configDriveKp(double kp) {
        // PID gains are feedback (FB) - only modify kP
        driveFBConfig.Slot0.kP = kp;
        driveMotor.getConfigurator().apply(driveFBConfig);
    }
    
    @Override
    public void configDriveKi(double ki) {
        // PID gains are feedback (FB) - only modify kI
        driveFBConfig.Slot0.kI = ki;
        driveMotor.getConfigurator().apply(driveFBConfig);
    }
    
    @Override
    public void configDriveKd(double kd) {
        // PID gains are feedback (FB) - only modify kD
        driveFBConfig.Slot0.kD = kd;
        driveMotor.getConfigurator().apply(driveFBConfig);
    }
    
    @Override
    public void configDriveFF(SimpleMotorFeedforward ff) {
        // Feedforward gains are separate from feedback
        driveFFConfig.Slot0.kS = ff.getKs();
        driveFFConfig.Slot0.kV = ff.getKv();
        driveFFConfig.Slot0.kA = ff.getKa();
        driveMotor.getConfigurator().apply(driveFFConfig);
    }
    
    @Override
    public void configDriveBrake(boolean isBrake) {
        // Brake/coast setting is separate from PID/FF
        driveBrakeConfig.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveBrakeConfig);
    }
    
    @Override
    public void configSteerKp(double kp) {
        // PID gains are feedback (FB) - only modify kP
        steerFBConfig.Slot0.kP = kp;
        steerMotor.getConfigurator().apply(steerFBConfig);
    }
    
    @Override
    public void configSteerKi(double ki) {
        // PID gains are feedback (FB) - only modify kI
        steerFBConfig.Slot0.kI = ki;
        steerMotor.getConfigurator().apply(steerFBConfig);
    }
    
    @Override
    public void configSteerKd(double kd) {
        // PID gains are feedback (FB) - only modify kD
        steerFBConfig.Slot0.kD = kd;
        steerMotor.getConfigurator().apply(steerFBConfig);
    }
    
    @Override
    public void configSteerBrake(boolean isBrake) {
        // Brake/coast setting is separate from PID
        steerBrakeConfig.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        steerMotor.getConfigurator().apply(steerBrakeConfig);
    }
    
    // ========== CONVERSION HELPER METHODS ==========
    
    /**
     * Convert drive motor rotations to mechanism radians (accounts for gear ratio)
     */
    private double driveMotorRotationsToMechanismRad(double motorRotations) {
        return Units.rotationsToRadians(motorRotations) / config.driveGearRatio;
    }
    
    /**
     * Convert steer motor rotations to mechanism radians (accounts for gear ratio)
     */
    private double steerMotorRotationsToMechanismRad(double motorRotations) {
        return Units.rotationsToRadians(motorRotations) / config.steerGearRatio;
    }
    
    /**
     * Convert mechanism radians to steer motor rotations (accounts for gear ratio)
     */
    private double mechanismRadToSteerMotorRotations(double mechanismRad) {
        return Units.radiansToRotations(mechanismRad) * config.steerGearRatio;
    }
    
    /**
     * Convert linear velocity to drive motor RPS (accounts for wheel diameter and gear ratio)
     */
    private double linearVelocityToDriveMotorRPS(double linearVelocityMPS) {
        double distancePerRotation = config.wheelDiameter.in(Meter) * Math.PI;
        return linearVelocityMPS / distancePerRotation * config.driveGearRatio;
    }
    
    /**
     * Convert drive motor RPS to linear velocity (accounts for wheel diameter and gear ratio)
     */
    private double driveMotorRPSToLinearVelocity(double motorRPS) {
        double distancePerRotation = config.wheelDiameter.in(Meter) * Math.PI;
        return motorRPS * distancePerRotation / config.driveGearRatio;
    }
    
    /**
     * Get wheel distance traveled from drive motor position
     */
    private double driveMotorPositionToWheelDistance(double motorPositionRad) {
        return motorPositionRad * config.wheelDiameter.in(Meter) * 0.5 / config.driveGearRatio;
    }
}
