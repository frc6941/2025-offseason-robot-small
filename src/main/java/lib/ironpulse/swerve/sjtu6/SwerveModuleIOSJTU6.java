package lib.ironpulse.swerve.sjtu6;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.SwerveModuleParamsNT;
import lib.ironpulse.swerve.SwerveConfig;
import lib.ironpulse.swerve.SwerveModuleIO;
import org.littletonrobotics.junction.Logger;

import java.util.Queue;
import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.units.Units.*;

public class SwerveModuleIOSJTU6 implements SwerveModuleIO {
    // sync thread for all modules
    private static final ReentrantLock syncLock = new ReentrantLock();
    //private static PhoenixSynchronizationThread syncThread;
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder encoder;
    private final SwerveSJTU6Config config;
    private final SwerveConfig.SwerveModuleConfig moduleConfig;
    // control requests
    private final VelocityTorqueCurrentFOC driveVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0);
    private final PositionTorqueCurrentFOC steerPositionRequest = new PositionTorqueCurrentFOC(0);
    private final VoltageOut steerVoltageRequest = new VoltageOut(0);
    // configuration objects - separate FB and FF to avoid overwriting settings
    private final TalonFXConfiguration driveFBConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration driveFFConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration steerFBConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration driveBrakeConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration steerBrakeConfig = new TalonFXConfiguration();
    // status signals
    private StatusSignal<Angle> drivePosition;
    private Queue<Double> drivePositionQueue;
    private StatusSignal<AngularVelocity> driveVelocity;
    private StatusSignal<Voltage> driveVoltage;
    private StatusSignal<Current> driveSupplyCurrentAmps;
    private StatusSignal<Current> driveTorqueCurrentAmps;
    private StatusSignal<Temperature> driveTemperatureCel;
    private StatusSignal<Angle> steerPosition;
    private Queue<Double> steerPositionQueue;
    private StatusSignal<AngularVelocity> steerVelocity;
    private StatusSignal<Voltage> steerVoltage;
    private StatusSignal<Current> steerSupplyCurrentAmps;
    private StatusSignal<Current> steerTorqueCurrentAmps;
    private StatusSignal<Temperature> steerTemperatureCel;

    private int moduleID;

    public SwerveModuleIOSJTU6(SwerveSJTU6Config config, int idx) {
        this.config = config;
        this.moduleConfig = config.moduleConfigs[idx];
        this.moduleID = idx;
//        if (syncThread == null)
//            syncThread = new PhoenixSynchronizationThread(syncLock, config.odometryFrequency);

        // initialize and config motors
        driveMotor = new TalonFX(moduleConfig.driveMotorId);
        steerMotor = new TalonFX(moduleConfig.steerMotorId);
        encoder = new CANcoder(moduleConfig.encoderId);
        configureDriveMotor();
        configureSteerMotor();

        // register signals, refresh in robotPeriodic
//        PhoenixUtils.registerSignals(
//                true,
//                drivePosition,
//                driveVelocity,
//                driveVoltage,
//                driveSupplyCurrentAmps,
//                driveTorqueCurrentAmps,
//                driveTemperatureCel,
//                steerPosition,
//                steerVelocity,
//                steerVoltage,
//                steerSupplyCurrentAmps,
//                steerTorqueCurrentAmps,
//                steerTemperatureCel
//        );

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = moduleConfig.encoderInverted?
            SensorDirectionValue.Clockwise_Positive:SensorDirectionValue.CounterClockwise_Positive;
        encoder.getConfigurator().apply(encoderConfig);

        driveMotor.clearStickyFaults();
        steerMotor.clearStickyFaults();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                drivePosition, driveVelocity, driveVoltage,
                driveSupplyCurrentAmps, driveTorqueCurrentAmps, driveTemperatureCel,
                steerPosition, steerVelocity, steerVoltage,
                steerSupplyCurrentAmps, steerTorqueCurrentAmps, steerTemperatureCel
        );

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
    }

//    public static void startSyncThread() {
//        if (syncThread != null && !syncThread.isAlive())
//            syncThread.start();
//    }

    private void configureDriveMotor() {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        // motor output
        driveConfig.MotorOutput.Inverted = moduleConfig.driveInverted ?
                InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // current limits - reasonable defaults for SJTU6
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = config.driveStatorCurrentLimit.in(Amp);

        // PID configuration - use defaults, will be updated by periodic calls
        driveConfig.Slot0.kP = SwerveModuleParamsNT.driveKp();
        driveConfig.Slot0.kI = SwerveModuleParamsNT.driveKi();
        driveConfig.Slot0.kD = SwerveModuleParamsNT.driveKd();
        driveConfig.Slot0.kS = SwerveModuleParamsNT.driveKs();
        driveConfig.Slot0.kV = SwerveModuleParamsNT.driveKv();
        driveConfig.Slot0.kA = SwerveModuleParamsNT.driveKa();

        // apply configuration
        //PhoenixUtils.tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));
        driveMotor.optimizeBusUtilization();

        // create signals
        drivePosition = driveMotor.getPosition();
        //drivePositionQueue = syncThread.registerSignal(drivePosition.clone());
        driveVelocity = driveMotor.getVelocity();
        driveVoltage = driveMotor.getMotorVoltage();
        driveSupplyCurrentAmps = driveMotor.getSupplyCurrent();
        driveTorqueCurrentAmps = driveMotor.getTorqueCurrent();
        driveTemperatureCel = driveMotor.getDeviceTemp();
    }

    private void configureSteerMotor() {
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        // motor output direction
        steerConfig.MotorOutput.Inverted = moduleConfig.steerInverted ?
                InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // encoder settings
        steerConfig.Feedback.FeedbackRemoteSensorID = moduleConfig.encoderId;
        steerConfig.Feedback.RotorToSensorRatio = config.steerGearRatio;
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        steerConfig.Feedback.withFeedbackRotorOffset(moduleConfig.steerMotorEncoderOffset);

        // current limits
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfig.CurrentLimits.StatorCurrentLimit = config.steerStatorCurrentLimit.in(Amp);

        // PID configuration - use defaults, will be updated by periodic calls
        steerConfig.Slot0.kP = SwerveModuleParamsNT.steerKp();
        steerConfig.Slot0.kI = SwerveModuleParamsNT.steerKi();
        steerConfig.Slot0.kD = SwerveModuleParamsNT.steerKd();

        // continuous wrap for steering
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

        // apply configuration
        //PhoenixUtils.tryUntilOk(5, () -> steerMotor.getConfigurator().apply(steerConfig, 0.25));
        steerMotor.optimizeBusUtilization();

        // create turn status signals
        steerPosition = steerMotor.getPosition();
        //steerPositionQueue = syncThread.registerSignal(steerPosition.clone());
        steerVelocity = steerMotor.getVelocity();
        steerVoltage = steerMotor.getMotorVoltage();
        steerSupplyCurrentAmps = steerMotor.getSupplyCurrent();
        steerTorqueCurrentAmps = steerMotor.getTorqueCurrent();
        steerTemperatureCel = steerMotor.getDeviceTemp();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                drivePosition, driveVelocity, driveVoltage,
                driveSupplyCurrentAmps, driveTorqueCurrentAmps, driveTemperatureCel,
                steerPosition, steerVelocity, steerVoltage,
                steerSupplyCurrentAmps, steerTorqueCurrentAmps, steerTemperatureCel
                );
        // drive motor inputs
        inputs.driveMotorConnected = BaseStatusSignal.isAllGood(
                drivePosition, driveVelocity, driveVoltage,
                driveSupplyCurrentAmps, driveTorqueCurrentAmps
        );
        inputs.driveMotorPositionRad = driveMotorRotationsToMechanismRad(drivePosition.getValueAsDouble());
        inputs.driveMotorVelocityRadPerSec = driveMotorRotationsToMechanismRad(driveVelocity.getValueAsDouble());
        inputs.driveMotorVoltageVolt = driveVoltage.getValueAsDouble();
        inputs.driveMotorSupplyCurrentAmpere = driveSupplyCurrentAmps.getValueAsDouble();
        inputs.driveMotorTorqueCurrentAmpere = driveTorqueCurrentAmps.getValueAsDouble();
        inputs.driveMotorTemperatureCel = driveTemperatureCel.getValueAsDouble();
        Logger.recordOutput("steerPosistion" + moduleID, driveMotorRotationsToMechanismRad(steerMotor.getPosition().getValueAsDouble()));

        // drive position samples
//        inputs.driveMotorPositionRadSamples = drivePositionQueue.stream().mapToDouble(
//                Units::rotationsToRadians).toArray();
//        drivePositionQueue.clear();

        // steer motor inputs
        inputs.steerMotorConnected = BaseStatusSignal.isAllGood(
                steerPosition, steerVelocity, steerVoltage,
                steerSupplyCurrentAmps, steerTorqueCurrentAmps
        );
        inputs.steerMotorPositionRad = driveMotorRotationsToMechanismRad(steerPosition.getValueAsDouble());
        inputs.steerMotorVelocityRadPerSec = driveMotorRotationsToMechanismRad(steerVelocity.getValueAsDouble());
        inputs.steerMotorVoltageVolt = steerVoltage.getValueAsDouble();
        inputs.steerMotorSupplyCurrentAmpere = steerSupplyCurrentAmps.getValueAsDouble();
        inputs.steerMotorTorqueCurrentAmpere = steerTorqueCurrentAmps.getValueAsDouble();
        inputs.steerMotorTemperatureCel = steerTemperatureCel.getValueAsDouble();

        // steer motor samples
//        inputs.steerMotorPositionRadSamples = steerPositionQueue.stream().mapToDouble(
//                Units::rotationsToRadians).toArray();
//        steerPositionQueue.clear();
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
        driveMotor.setControl(driveVelocityRequest.withVelocity(velocityRps));
    }

    @Override
    public void setDriveVelocity(LinearVelocity velocity, Current ff) {
        double velocityRps = linearVelocityToDriveMotorRPS(velocity.in(MetersPerSecond));
        driveMotor.setControl(
                driveVelocityRequest.withVelocity(velocityRps).withFeedForward(ff.in(Amp))
        );
    }

    @Override
    public void setSteerOpenLoop(Voltage voltage) {
        steerMotor.setControl(steerVoltageRequest.withOutput(voltage.in(Volt)));
    }

    @Override
    public void setSteerAngleAbsolute(Angle angle) {
        double positionRotations = mechanismRadToSteerMotorRotations(angle.in(Radian));
        steerMotor.setControl(steerPositionRequest.withPosition(positionRotations));
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
