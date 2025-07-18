package lib.ironpulse.swerve.sjtu6;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import lib.ironpulse.swerve.SwerveConfig;
import lib.ironpulse.swerve.SwerveModuleIO;
import lib.ironpulse.utils.PhoenixSynchronizationThread;
import lib.ironpulse.utils.PhoenixUtils;

import java.util.ArrayDeque;
import java.util.Queue;
import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.units.Units.*;

public class SwerveModuleIOSJTU6 implements SwerveModuleIO {
    // sync thread for all modules
    private static final ReentrantLock syncLock = new ReentrantLock();
    private static PhoenixSynchronizationThread syncThread;
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder encoder;
    private final SwerveSJTU6Config config;
    private final SwerveConfig.SwerveModuleConfig moduleConfig;
    // control requests
    private final VelocityTorqueCurrentFOC driveVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0);
    private final PositionDutyCycle steerPositionRequest = new PositionDutyCycle(0);
    private final VoltageOut steerVoltageRequest = new VoltageOut(0);
    // configuration objects
    private final TalonFXConfiguration driveControlConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration steerControlConfig = new TalonFXConfiguration();

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


    private Double recordSteerCommand = null;
    private int moduleID;


    public SwerveModuleIOSJTU6(SwerveSJTU6Config config, int idx) {
        this.config = config;
        this.moduleConfig = config.moduleConfigs[idx];
        this.moduleID = idx;

        if (syncThread == null)
            syncThread = new PhoenixSynchronizationThread(syncLock, config.odometryFrequency);

        // initialize and config motors
        driveMotor = new TalonFX(moduleConfig.driveMotorId, config.canivoreCanBusName);
        steerMotor = new TalonFX(moduleConfig.steerMotorId, config.canivoreCanBusName);
        encoder = new CANcoder(moduleConfig.encoderId, config.canivoreCanBusName);
        configureDriveMotor();
        configureSteerMotor();

        // register signals, refresh in robotPeriodic
        PhoenixUtils.registerSignals(
                true,
                drivePosition,
                driveVelocity,
                driveVoltage,
                driveSupplyCurrentAmps,
                driveTorqueCurrentAmps,
                driveTemperatureCel,
                steerPosition,
                steerVelocity,
                steerVoltage,
                steerSupplyCurrentAmps,
                steerTorqueCurrentAmps,
                steerTemperatureCel
        );

        driveMotor.clearStickyFaults();
        steerMotor.clearStickyFaults();

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
    }

    public static void startSyncThread() {
        if (syncThread != null && !syncThread.isAlive())
            syncThread.start();
    }

    // Getters for shared sync resources (used by IMU to ensure synchronized sampling)
    public static ReentrantLock getSyncLock() {
        return syncLock;
    }

    public static PhoenixSynchronizationThread getSyncThread() {
        return syncThread;
    }


    private void configureDriveMotor() {
        // motor output
        driveControlConfig.MotorOutput.Inverted = moduleConfig.driveInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        driveControlConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // current limits - reasonable defaults for SJTU6
        driveControlConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveControlConfig.CurrentLimits.StatorCurrentLimit = config.driveStatorCurrentLimit.in(Amp);

        // apply configuration
        PhoenixUtils.tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveControlConfig, 0.25));
        driveMotor.getConfigurator().apply(driveControlConfig);
        driveMotor.optimizeBusUtilization();

        // create signals
        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveVoltage = driveMotor.getMotorVoltage();
        driveSupplyCurrentAmps = driveMotor.getSupplyCurrent();
        driveTorqueCurrentAmps = driveMotor.getTorqueCurrent();
        driveTemperatureCel = driveMotor.getDeviceTemp();

        // Configure signal update frequencies to prevent stale messages
        // High priority signals for control (100Hz = 10ms)
        drivePosition.setUpdateFrequency(100.0);
        driveVelocity.setUpdateFrequency(100.0);

        // Medium priority signals for telemetry (50Hz = 20ms)
        driveVoltage.setUpdateFrequency(50.0);
        driveSupplyCurrentAmps.setUpdateFrequency(50.0);
        driveTorqueCurrentAmps.setUpdateFrequency(50.0);

        // Low priority signals for diagnostics (10Hz = 100ms)
        driveTemperatureCel.setUpdateFrequency(10.0);

        // Initialize position sampling queues
        if (syncThread != null && drivePosition != null) {
            drivePositionQueue = syncThread.registerSignal(drivePosition.clone());
        }
        if (drivePositionQueue == null) {
            drivePositionQueue = new ArrayDeque<>();
        }
    }

    private void configureSteerMotor() {
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        encoderConfig.MagnetSensor.SensorDirection = moduleConfig.encoderInverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = moduleConfig.steerMotorEncoderOffset.magnitude();


        // motor output direction
        steerControlConfig.MotorOutput.Inverted = moduleConfig.steerInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        steerControlConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // encoder settings
        steerControlConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        steerControlConfig.Feedback.FeedbackRemoteSensorID = moduleConfig.encoderId;
        steerControlConfig.Feedback.RotorToSensorRatio = config.steerGearRatio;

        // current limits
        steerControlConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerControlConfig.CurrentLimits.StatorCurrentLimit = config.steerStatorCurrentLimit.in(Amp);

        // PID configuration - use defaults, will be updated by periodic calls
        steerControlConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        // continuous wrap for steering
        steerControlConfig.ClosedLoopGeneral.ContinuousWrap = true;

        // apply configuration
        PhoenixUtils.tryUntilOk(5, () -> steerMotor.getConfigurator().apply(steerControlConfig, 0.25));
        steerMotor.getConfigurator().apply(steerControlConfig);
        encoder.getConfigurator().apply(encoderConfig);


        // create turn status signals
        steerPosition = steerMotor.getPosition();
        steerVelocity = steerMotor.getVelocity();
        steerVoltage = steerMotor.getMotorVoltage();
        steerSupplyCurrentAmps = steerMotor.getSupplyCurrent();
        steerTorqueCurrentAmps = steerMotor.getTorqueCurrent();
        steerTemperatureCel = steerMotor.getDeviceTemp();

        // Configure signal update frequencies to prevent stale messages
        // High priority signals for control (100Hz = 10ms)
        steerPosition.setUpdateFrequency(config.odometryFrequency);
        steerVelocity.setUpdateFrequency(config.odometryFrequency);

        // Medium priority signals for telemetry (50Hz = 20ms)
        steerVoltage.setUpdateFrequency(50.0);
        steerSupplyCurrentAmps.setUpdateFrequency(50.0);
        steerTorqueCurrentAmps.setUpdateFrequency(50.0);

        // Low priority signals for diagnostics (10Hz = 100ms)
        steerTemperatureCel.setUpdateFrequency(10.0);

        // Initialize position sampling queues
        if (syncThread != null && drivePosition != null) {
            drivePositionQueue = syncThread.registerSignal(drivePosition.clone());
        }
        if (drivePositionQueue == null) {
            drivePositionQueue = new ArrayDeque<>();
        }

        if (syncThread != null && steerPosition != null) {
            steerPositionQueue = syncThread.registerSignal(steerPosition.clone());
        }
        if (steerPositionQueue == null) {
            steerPositionQueue = new ArrayDeque<>();
        }
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // drive motor inputs
        inputs.driveMotorConnected = BaseStatusSignal.isAllGood(
                drivePosition, driveVelocity, driveVoltage,
                driveSupplyCurrentAmps, driveTorqueCurrentAmps
        );
        inputs.driveMotorPositionRad = driveMotorRotationsToMechanismRad(drivePosition.getValueAsDouble());
        inputs.driveMotorVelocityRadPerSec = driveMotorRotationsPerSecToMechanismRadPerSec(
                driveVelocity.getValueAsDouble());
        inputs.driveMotorVoltageVolt = driveVoltage.getValueAsDouble();
        inputs.driveMotorSupplyCurrentAmpere = driveSupplyCurrentAmps.getValueAsDouble();
        inputs.driveMotorTorqueCurrentAmpere = driveTorqueCurrentAmps.getValueAsDouble();
        inputs.driveMotorTemperatureCel = driveTemperatureCel.getValueAsDouble();

        // drive position samples
        if (drivePositionQueue != null && !drivePositionQueue.isEmpty()) {
            inputs.driveMotorPositionRadSamples = drivePositionQueue.stream().mapToDouble(
                    this::driveMotorRotationsToMechanismRad).toArray();
            drivePositionQueue.clear();
        } else {
            inputs.driveMotorPositionRadSamples = new double[]{inputs.driveMotorPositionRad};
        }

        //Logger.recordOutput("steerPosition" + moduleID, steerMotorRotationsToMechanismRad(steerMotor.getPosition().getValueAsDouble()));


        // steer motor inputs
        inputs.steerMotorConnected = BaseStatusSignal.isAllGood(
                steerPosition, steerVelocity, steerVoltage,
                steerSupplyCurrentAmps, steerTorqueCurrentAmps
        );
        inputs.steerMotorPositionRad = steerMotorRotationsToMechanismRad(steerPosition.getValueAsDouble());
        inputs.steerMotorVelocityRadPerSec = steerMotorRotationsPerSecToMechanismRadPerSec(
                steerVelocity.getValueAsDouble());
        inputs.steerMotorVoltageVolt = steerVoltage.getValueAsDouble();
        inputs.steerMotorSupplyCurrentAmpere = steerSupplyCurrentAmps.getValueAsDouble();
        inputs.steerMotorTorqueCurrentAmpere = steerTorqueCurrentAmps.getValueAsDouble();
        inputs.steerMotorTemperatureCel = steerTemperatureCel.getValueAsDouble();

        // steer motor samples  
        if (steerPositionQueue != null && !steerPositionQueue.isEmpty()) {
            inputs.steerMotorPositionRadSamples = steerPositionQueue.stream().mapToDouble(
                    this::steerMotorRotationsToMechanismRad).toArray();
            steerPositionQueue.clear();
        } else {
            inputs.steerMotorPositionRadSamples = new double[]{inputs.steerMotorPositionRad};
        }

        // Ensure both sample arrays have the same length for safety
        int driveLength = inputs.driveMotorPositionRadSamples.length;
        int steerLength = inputs.steerMotorPositionRadSamples.length;
        if (driveLength != steerLength) {
            //TODO: maybe change the way we handle the array? this might cause gitter
            // Use the current position for both if lengths don't match
            inputs.driveMotorPositionRadSamples = new double[]{inputs.driveMotorPositionRad};
            inputs.steerMotorPositionRadSamples = new double[]{inputs.steerMotorPositionRad};
        }

    }


    @Override
    public void setSwerveModuleState(SwerveModuleState state) {
        // Set drive velocity
        double velocityRps = linearVelocityToWheelRPS(MathUtil.applyDeadband(state.speedMetersPerSecond, 0.02));
        driveMotor.setControl(driveVelocityRequest.withVelocity(velocityRps * config.driveGearRatio));

        // Set steer angle
        double positionRotations = mechanismRadToSteerMotorRotations(state.angle.getRadians());
        if (state.speedMetersPerSecond < 0.02) {
            if (recordSteerCommand == null) {
                // first entry on low speed cap, record angle command
                recordSteerCommand = positionRotations;
            }
        } else {
            recordSteerCommand = null;
        }

        steerMotor.setControl(steerPositionRequest.withPosition(
                recordSteerCommand != null ? recordSteerCommand : positionRotations));
    }

    @Override
    public void setDriveOpenLoop(Voltage voltage) {
        driveMotor.setControl(driveVoltageRequest.withOutput(voltage.in(Volt)));
    }

    @Override
    public void setDriveVelocity(LinearVelocity velocity) {
        double velocityRps = linearVelocityToWheelRPS(velocity.in(MetersPerSecond));
        driveMotor.setControl(driveVelocityRequest.withVelocity(velocityRps * config.driveGearRatio));
    }

    @Override
    public void setDriveVelocity(LinearVelocity velocity, Current ff) {
        double velocityRps = linearVelocityToWheelRPS(velocity.in(MetersPerSecond));
        driveMotor.setControl(
                driveVelocityRequest.withVelocity(velocityRps * config.driveGearRatio).withFeedForward(ff.in(Amp)));
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
    public void configDriveController(double kp, double ki, double kd, double ks, double kv, double ka) {
        // Configure both PID and FF parameters in single config to avoid overwriting
        driveControlConfig.Slot0.kP = kp;
        driveControlConfig.Slot0.kI = ki;
        driveControlConfig.Slot0.kD = kd;
        driveControlConfig.Slot0.kS = ks;
        driveControlConfig.Slot0.kV = kv;
        driveControlConfig.Slot0.kA = ka;

        // Apply single combined configuration
        driveMotor.getConfigurator().apply(driveControlConfig);
    }


    @Override
    public void configDriveBrake(boolean isBrake) {
        // Brake/coast setting is separate from PID/FF
        driveControlConfig.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveControlConfig);
    }

    @Override
    public void configSteerController(double kp, double ki, double kd, double ks) {
        // PID gains and static friction feedforward - set all parameters at once
        steerControlConfig.Slot0.kP = kp;
        steerControlConfig.Slot0.kI = ki;
        steerControlConfig.Slot0.kD = kd;
        steerControlConfig.Slot0.kS = ks;
        steerMotor.getConfigurator().apply(steerControlConfig);
    }

    @Override
    public void configSteerBrake(boolean isBrake) {
        // Brake/coast setting is separate from PID
        steerControlConfig.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        steerMotor.getConfigurator().apply(steerControlConfig);
    }

    // ========== UNIT CONVERSION METHODS ==========

    /*
     * CONVERSION OVERVIEW:
     *
     * This swerve module uses TalonFX motors with gear reduction for both drive and steering.
     * The interface expects wheel/mechanism units, but the motors need motor shaft units.
     *
     * Drive System:
     * - Motor shaft -> [gear ratio] -> Wheel
     * - Higher gear ratio = motor spins faster than wheel
     * - Example: 6.14:1 gear ratio means motor rotates 6.14 times per wheel rotation
     *
     * Steer System:
     * - Motor shaft -> [gear ratio] -> Module rotation
     * - CANcoder provides absolute position feedback fused with motor encoder
     * - For SJTU6, typically 1:1 (direct drive) so no gear ratio conversion needed
     */

    // ========== DRIVE MOTOR CONVERSIONS ==========

    /**
     * Convert drive motor rotations to wheel position in radians.
     * <p>
     * Flow: Motor rotations -> Wheel radians
     * Math: motor_rot * (2π rad/rot) / gear_ratio = wheel_rad
     *
     * @param motorRotations Raw motor encoder rotations
     * @return Wheel position in radians
     */
    private double driveMotorRotationsToMechanismRad(double motorRotations) {
        return Units.rotationsToRadians(motorRotations) / config.driveGearRatio;
    }

    /**
     * Convert drive motor rotations per second to wheel angular velocity in rad/s.
     * <p>
     * Flow: Motor RPS -> Wheel rad/s
     * Math: motor_rps * (2π rad/rot) / gear_ratio = wheel_rad_per_sec
     *
     * @param motorRotationsPerSec Raw motor velocity in rotations per second
     * @return Wheel angular velocity in rad/s
     */
    private double driveMotorRotationsPerSecToMechanismRadPerSec(double motorRotationsPerSec) {
        return Units.rotationsToRadians(motorRotationsPerSec) / config.driveGearRatio;
    }

    /**
     * Convert linear velocity to wheel rotations per second.
     * <p>
     * Flow: Linear velocity (m/s) -> Wheel RPS
     * Math: linear_vel / (wheel_diameter * π) = wheel_rps
     * <p>
     * This is used at the interface level - gear ratio is applied when commanding motors.
     *
     * @param linearVelocityMPS Linear velocity in meters per second
     * @return Wheel rotations per second
     */
    private double linearVelocityToWheelRPS(double linearVelocityMPS) {
        double wheelCircumference = config.wheelDiameter.in(Meter) * Math.PI;
        return linearVelocityMPS / wheelCircumference;
    }

    // ========== STEER MOTOR CONVERSIONS ==========

    /**
     * Convert steer motor rotations to mechanism angle in radians.
     * <p>
     * Flow: Motor rotations -> Module angle radians
     * Math: motor_rot * (2π rad/rot) = mechanism_rad
     * <p>
     * Note: SJTU6 typically uses 1:1 gearing (direct drive), so no gear ratio needed.
     * The CANcoder is fused with the motor encoder to provide absolute positioning.
     *
     * @param motorRotations Raw motor encoder rotations
     * @return Module angle in radians
     */
    private double steerMotorRotationsToMechanismRad(double motorRotations) {
        return Units.rotationsToRadians(motorRotations);
    }

    /**
     * Convert steer motor rotations per second to mechanism angular velocity in rad/s.
     * <p>
     * Flow: Motor RPS -> Module angular velocity rad/s
     * Math: motor_rps * (2π rad/rot) = mechanism_rad_per_sec
     *
     * @param motorRotationsPerSec Raw motor velocity in rotations per second
     * @return Module angular velocity in rad/s
     */
    private double steerMotorRotationsPerSecToMechanismRadPerSec(double motorRotationsPerSec) {
        return Units.rotationsToRadians(motorRotationsPerSec);
    }

    /**
     * Convert mechanism angle in radians to steer motor rotations.
     * <p>
     * Flow: Module angle radians -> Motor rotations
     * Math: mechanism_rad / (2π rad/rot) = motor_rot
     *
     * @param mechanismRad Desired module angle in radians
     * @return Motor position in rotations
     */
    private double mechanismRadToSteerMotorRotations(double mechanismRad) {
        return Units.radiansToRotations(mechanismRad);
    }
}
