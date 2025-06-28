package lib.ironpulse.swerve.sjtu6;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.SwerveModuleParamsNT;
import lib.ironpulse.swerve.SwerveConfig;
import lib.ironpulse.swerve.SwerveModuleIO;
import lib.ironpulse.utils.PhoenixSynchronizationThread;
import lib.ironpulse.utils.PhoenixUtils;
import org.littletonrobotics.junction.Logger;

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

    // Per-module cached tunable values to prevent unnecessary configuration updates
    private double lastDriveKp = Double.NaN;
    private double lastDriveKi = Double.NaN;
    private double lastDriveKd = Double.NaN;
    private double lastDriveKs = Double.NaN;
    private double lastDriveKv = Double.NaN;
    private double lastDriveKa = Double.NaN;
    private boolean lastDriveIsBrake = true;
    
    private double lastSteerKp = Double.NaN;
    private double lastSteerKi = Double.NaN;
    private double lastSteerKd = Double.NaN;
    private double lastSteerKs = Double.NaN;
    private boolean lastSteerIsBrake = true;

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
        PhoenixUtils.tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));
        driveMotor.getConfigurator().apply(driveConfig);
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
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        encoderConfig.MagnetSensor.SensorDirection = moduleConfig.encoderInverted?
        SensorDirectionValue.Clockwise_Positive:SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = moduleConfig.steerMotorEncoderOffset.magnitude();
        

        // motor output direction
        steerConfig.MotorOutput.Inverted = moduleConfig.steerInverted ?
                InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // encoder settings
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        steerConfig.Feedback.FeedbackRemoteSensorID = moduleConfig.encoderId;
        steerConfig.Feedback.RotorToSensorRatio = config.steerGearRatio;

        // current limits
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        steerConfig.CurrentLimits.StatorCurrentLimit = config.steerStatorCurrentLimit.in(Amp);

        // PID configuration - use defaults, will be updated by periodic calls
        steerConfig.Slot0.kP = SwerveModuleParamsNT.steerKp();
        steerConfig.Slot0.kI = SwerveModuleParamsNT.steerKi();
        steerConfig.Slot0.kD = SwerveModuleParamsNT.steerKd();
        steerConfig.Slot0.kS = SwerveModuleParamsNT.steerKs();
        steerConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;


        // continuous wrap for steering
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

        // apply configuration
        PhoenixUtils.tryUntilOk(5, () -> steerMotor.getConfigurator().apply(steerConfig, 0.25));
        steerMotor.getConfigurator().apply(steerConfig);
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
        steerPosition.setUpdateFrequency(100.0);
        steerVelocity.setUpdateFrequency(100.0);
        
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
        inputs.driveMotorVelocityRadPerSec = driveMotorRotationsPerSecToMechanismRadPerSec(driveVelocity.getValueAsDouble());
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

        Logger.recordOutput("steerPosition" + moduleID, steerMotorRotationsToMechanismRad(steerMotor.getPosition().getValueAsDouble()));


        // steer motor inputs
        inputs.steerMotorConnected = BaseStatusSignal.isAllGood(
                steerPosition, steerVelocity, steerVoltage,
                steerSupplyCurrentAmps, steerTorqueCurrentAmps
        );
        inputs.steerMotorPositionRad = steerMotorRotationsToMechanismRad(steerPosition.getValueAsDouble());
        inputs.steerMotorVelocityRadPerSec = steerMotorRotationsPerSecToMechanismRadPerSec(steerVelocity.getValueAsDouble());
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
            // Use the current position for both if lengths don't match
            inputs.driveMotorPositionRadSamples = new double[]{inputs.driveMotorPositionRad};
            inputs.steerMotorPositionRadSamples = new double[]{inputs.steerMotorPositionRad};
        }

        // Update PID values only when they change (per-module change detection)
        if (Constants.kTuning) {
            updateTunableValuesIfChanged();
        }

    }

    /**
     * Updates tunable PID values only when they have changed for this specific module instance.
     * This prevents unnecessary motor controller configuration calls.
     */
    private void updateTunableValuesIfChanged() {
        boolean driveSlot0Changed = false;
        boolean steerSlot0Changed = false;
        boolean driveBrakeConfigChanged = false;
        boolean steerBrakeConfigChanged = false;

        // Check all drive slot0 parameters (PID + feedforward)
        double currentDriveKp = SwerveModuleParamsNT.driveKp();
        double currentDriveKi = SwerveModuleParamsNT.driveKi();
        double currentDriveKd = SwerveModuleParamsNT.driveKd();
        double currentDriveKs = SwerveModuleParamsNT.driveKs();
        double currentDriveKv = SwerveModuleParamsNT.driveKv();
        double currentDriveKa = SwerveModuleParamsNT.driveKa();
        
        if (currentDriveKp != lastDriveKp) {
            lastDriveKp = currentDriveKp;
            driveSlot0Changed = true;
        }
        if (currentDriveKi != lastDriveKi) {
            lastDriveKi = currentDriveKi;
            driveSlot0Changed = true;
        }
        if (currentDriveKd != lastDriveKd) {
            lastDriveKd = currentDriveKd;
            driveSlot0Changed = true;
        }
        if (currentDriveKs != lastDriveKs) {
            lastDriveKs = currentDriveKs;
            driveSlot0Changed = true;
        }
        if (currentDriveKv != lastDriveKv) {
            lastDriveKv = currentDriveKv;
            driveSlot0Changed = true;
        }
        if (currentDriveKa != lastDriveKa) {
            lastDriveKa = currentDriveKa;
            driveSlot0Changed = true;
        }

        // Check all steer slot0 parameters (PID + kS feedforward)
        double currentSteerKp = SwerveModuleParamsNT.steerKp();
        double currentSteerKi = SwerveModuleParamsNT.steerKi();
        double currentSteerKd = SwerveModuleParamsNT.steerKd();
        double currentSteerKs = SwerveModuleParamsNT.steerKs();
        
        if (currentSteerKp != lastSteerKp) {
            lastSteerKp = currentSteerKp;
            steerSlot0Changed = true;
        }
        if (currentSteerKi != lastSteerKi) {
            lastSteerKi = currentSteerKi;
            steerSlot0Changed = true;
        }
        if (currentSteerKd != lastSteerKd) {
            lastSteerKd = currentSteerKd;
            steerSlot0Changed = true;
        }
        if (currentSteerKs != lastSteerKs) {
            lastSteerKs = currentSteerKs;
            steerSlot0Changed = true;
        }

        // Check brake modes
        boolean currentDriveIsBrake = SwerveModuleParamsNT.driveIsBrake();
        boolean currentSteerIsBrake = SwerveModuleParamsNT.steerIsBrake();
        
        if (currentDriveIsBrake != lastDriveIsBrake) {
            driveBrakeConfig.MotorOutput.NeutralMode = currentDriveIsBrake ? 
                NeutralModeValue.Brake : NeutralModeValue.Coast;
            lastDriveIsBrake = currentDriveIsBrake;
            driveBrakeConfigChanged = true;
        }
        if (currentSteerIsBrake != lastSteerIsBrake) {
            steerBrakeConfig.MotorOutput.NeutralMode = currentSteerIsBrake ? 
                NeutralModeValue.Brake : NeutralModeValue.Coast;
            lastSteerIsBrake = currentSteerIsBrake;
            steerBrakeConfigChanged = true;
        }

        // Apply only the specific configurations that changed to avoid overwriting other settings
        // Combine all drive slot0 parameters (PID + feedforward) into single application
        if (driveSlot0Changed) {
            var driveSlot0Config = new com.ctre.phoenix6.configs.Slot0Configs();
            // Always set all parameters to current values (from either cache or current NT values)
            driveSlot0Config.kP = SwerveModuleParamsNT.driveKp();
            driveSlot0Config.kI = SwerveModuleParamsNT.driveKi();
            driveSlot0Config.kD = SwerveModuleParamsNT.driveKd();
            driveSlot0Config.kS = SwerveModuleParamsNT.driveKs();
            driveSlot0Config.kV = SwerveModuleParamsNT.driveKv();
            driveSlot0Config.kA = SwerveModuleParamsNT.driveKa();
            driveSlot0Config.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
            driveMotor.getConfigurator().apply(driveSlot0Config);
        }
        if (steerSlot0Changed) {
            // Apply all steer slot0 parameters including kS
            var steerSlot0Config = new com.ctre.phoenix6.configs.Slot0Configs();
            steerSlot0Config.kP = SwerveModuleParamsNT.steerKp();
            steerSlot0Config.kI = SwerveModuleParamsNT.steerKi();
            steerSlot0Config.kD = SwerveModuleParamsNT.steerKd();
            steerSlot0Config.kS = SwerveModuleParamsNT.steerKs();
            steerSlot0Config.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
            steerMotor.getConfigurator().apply(steerSlot0Config);
        }
        if (driveBrakeConfigChanged) {
            // Only apply the motor output configuration
            var driveMotorOutputConfig = new com.ctre.phoenix6.configs.MotorOutputConfigs();
            driveMotorOutputConfig.NeutralMode = driveBrakeConfig.MotorOutput.NeutralMode;
            driveMotor.getConfigurator().apply(driveMotorOutputConfig);
        }
        if (steerBrakeConfigChanged) {
            // Only apply the motor output configuration
            var steerMotorOutputConfig = new com.ctre.phoenix6.configs.MotorOutputConfigs();
            steerMotorOutputConfig.NeutralMode = steerBrakeConfig.MotorOutput.NeutralMode;
            steerMotor.getConfigurator().apply(steerMotorOutputConfig);
        }
    }

    @Override
    public void setSwerveModuleState(SwerveModuleState state) {
        // Set drive velocity
        double velocityRps = linearVelocityToWheelRPS(state.speedMetersPerSecond);
        driveMotor.setControl(driveVelocityRequest.withVelocity(velocityRps * config.driveGearRatio));

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
        double velocityRps = linearVelocityToWheelRPS(velocity.in(MetersPerSecond));
        driveMotor.setControl(driveVelocityRequest.withVelocity(velocityRps * config.driveGearRatio));
    }

    @Override
    public void setDriveVelocity(LinearVelocity velocity, Current ff) {
        double velocityRps = linearVelocityToWheelRPS(velocity.in(MetersPerSecond));
        driveMotor.setControl(
                driveVelocityRequest.withVelocity(velocityRps * config.driveGearRatio).withFeedForward(ff.in(Amp))
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
    public void configSteerKs(double ks) {
        // Static friction feedforward - only modify kS
        steerFBConfig.Slot0.kS = ks;
        steerMotor.getConfigurator().apply(steerFBConfig);
    }

    @Override
    public void configSteerBrake(boolean isBrake) {
        // Brake/coast setting is separate from PID
        steerBrakeConfig.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        steerMotor.getConfigurator().apply(steerBrakeConfig);
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
     * 
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
     * 
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
     * 
     * Flow: Linear velocity (m/s) -> Wheel RPS
     * Math: linear_vel / (wheel_diameter * π) = wheel_rps
     * 
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
     * 
     * Flow: Motor rotations -> Module angle radians
     * Math: motor_rot * (2π rad/rot) = mechanism_rad
     * 
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
     * 
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
     * 
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
