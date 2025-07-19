package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;

import static frc.robot.Constants.Elevator.*;
import static frc.robot.ElevatorCommonNT.*;
import static frc.robot.Ports.ELEVATOR_FOLLOWER;
import static frc.robot.Ports.ELEVATOR_MAIN;

public class ElevatorIOReal implements ElevatorIO {
    // Hardware
    private final TalonFX leader;
    private final TalonFX follower;

    // Configurators
    private final TalonFXConfigurator leaderConfigurator;
    private final TalonFXConfigurator followerConfigurator;

    private final Slot0Configs slot0Configs;
    private final MotionMagicConfigs motionMagicConfigs;

    private final DynamicMotionMagicVoltage motionRequest = new DynamicMotionMagicVoltage(0.0, 100, 300, 0).withEnableFOC(true);

    private final StatusSignal<AngularVelocity> velocityLeft;
    private final StatusSignal<Angle> positionLeft;
    private final StatusSignal<Voltage> voltageLeft;
    private final StatusSignal<Current> statorLeft;
    private final StatusSignal<Current> supplyLeft;
    private final StatusSignal<Temperature> tempLeft;
    private double setpointMeters = 0;
    private boolean isGoingUp = false;

    public ElevatorIOReal() {
        this.leader = new TalonFX(ELEVATOR_MAIN.id, ELEVATOR_MAIN.bus);
        this.follower = new TalonFX(ELEVATOR_FOLLOWER.id, ELEVATOR_FOLLOWER.bus);

        this.leaderConfigurator = leader.getConfigurator();
        this.followerConfigurator = follower.getConfigurator();

        // Configs
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

        leader.setPosition(heightToTalonPos(ELEVATOR_DEFAULT_POSITION_WHEN_DISABLED));
        follower.setPosition(heightToTalonPos(ELEVATOR_DEFAULT_POSITION_WHEN_DISABLED));

        MotorOutputConfigs leaderMotorConfigs = new MotorOutputConfigs();
        leaderMotorConfigs.NeutralMode = NeutralModeValue.Brake;
        leaderMotorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        MotorOutputConfigs followerMotorConfigs = new MotorOutputConfigs();
        followerMotorConfigs.NeutralMode = NeutralModeValue.Brake;

        // Initialize motion magic configs (not used for Dynamic Motion Magic, but kept for compatibility)
        motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration = motionAccelerationUp.getValue(); // Default fallback
        motionMagicConfigs.MotionMagicCruiseVelocity = motionCruiseVelocityUp.getValue(); // Default fallback
        motionMagicConfigs.MotionMagicJerk = motionJerkUp.getValue(); // Default fallback

        // Set default Dynamic Motion Magic parameters (will be overridden in setElevatorTarget)
        motionRequest.Velocity = motionCruiseVelocityUp.getValue();
        motionRequest.Acceleration = motionAccelerationUp.getValue();
        motionRequest.Jerk = motionJerkUp.getValue();

        slot0Configs = new Slot0Configs();
        slot0Configs.kA = ElevatorGainsClass.ELEVATOR_KA.getValue();
        slot0Configs.kS = ElevatorGainsClass.ELEVATOR_KS.getValue();
        slot0Configs.kV = ElevatorGainsClass.ELEVATOR_KV.getValue();
        slot0Configs.kG = ElevatorGainsClass.ELEVATOR_KG.getValue();
        slot0Configs.kP = ElevatorGainsClass.ELEVATOR_KP.getValue();
        slot0Configs.kI = ElevatorGainsClass.ELEVATOR_KI.getValue();
        slot0Configs.kD = ElevatorGainsClass.ELEVATOR_KD.getValue();

        //Since elevator don't start at zero, not needed
        //resetElevatorPosition();

        leaderConfigurator.apply(currentLimitsConfigs);
        leaderConfigurator.apply(leaderMotorConfigs);
        leaderConfigurator.apply(slot0Configs);
        leaderConfigurator.apply(motionMagicConfigs);
        followerConfigurator.apply(currentLimitsConfigs);
        followerConfigurator.apply(followerMotorConfigs);
        followerConfigurator.apply(slot0Configs);
        followerConfigurator.apply(motionMagicConfigs);

        leader.clearStickyFaults();
        follower.clearStickyFaults();

        velocityLeft = leader.getVelocity();
        positionLeft = leader.getPosition();
        voltageLeft = leader.getSupplyVoltage();
        statorLeft = leader.getStatorCurrent();
        supplyLeft = leader.getSupplyCurrent();
        tempLeft = leader.getDeviceTemp();

        follower.setControl(new Follower(leader.getDeviceID(), true));

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                velocityLeft,
                positionLeft,
                voltageLeft,
                statorLeft,
                supplyLeft,
                tempLeft
        );

        inputs.positionMeters = talonPosToHeight(leader.getPosition().getValueAsDouble());
        inputs.setpointMeters = setpointMeters;
        inputs.velocityMetersPerSec = getElevatorVelocity();
        inputs.appliedVolts = voltageLeft.getValueAsDouble();
        inputs.statorCurrentAmps = statorLeft.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyLeft.getValueAsDouble();
        inputs.tempCelsius = tempLeft.getValueAsDouble();
        inputs.motorVoltage = leader.getMotorVoltage().getValueAsDouble();
        // Dynamic Motion Magic logging
        inputs.isGoingUp = isGoingUp;
        inputs.currentAcceleration = isGoingUp ? motionAccelerationUp.getValue() : motionAccelerationDown.getValue();
        inputs.currentCruiseVelocity = isGoingUp ? motionCruiseVelocityUp.getValue() : motionCruiseVelocityDown.getValue();
        inputs.currentJerk = isGoingUp ? motionJerkUp.getValue() : motionJerkDown.getValue();
        if (ElevatorGainsClass.isAnyChanged()) {
            slot0Configs.kA = ElevatorGainsClass.ELEVATOR_KA.getValue();
            slot0Configs.kS = ElevatorGainsClass.ELEVATOR_KS.getValue();
            slot0Configs.kV = ElevatorGainsClass.ELEVATOR_KV.getValue();
            slot0Configs.kP = ElevatorGainsClass.ELEVATOR_KP.getValue();
            slot0Configs.kI = ElevatorGainsClass.ELEVATOR_KI.getValue();
            slot0Configs.kD = ElevatorGainsClass.ELEVATOR_KD.getValue();
            slot0Configs.kG = ElevatorGainsClass.ELEVATOR_KG.getValue();

            leaderConfigurator.apply(slot0Configs);
            followerConfigurator.apply(slot0Configs);

            // Update Dynamic Motion Magic parameters in real-time during tuning
            if (isGoingUp) {
                motionRequest.Velocity = motionCruiseVelocityUp.getValue();
                motionRequest.Acceleration = motionAccelerationUp.getValue();
                motionRequest.Jerk = motionJerkUp.getValue();
            } else {
                motionRequest.Velocity = motionCruiseVelocityDown.getValue();
                motionRequest.Acceleration = motionAccelerationDown.getValue();
                motionRequest.Jerk = motionJerkDown.getValue();
            }
        }
    }

    @Override
    public void setElevatorVoltage(double volts) {
        leader.setControl(new VoltageOut(volts));
    }

    @Override
    public void setElevatorTarget(double meters) {
        // Default implementation - use last known direction or assume up for safety
        setElevatorTarget(meters, isGoingUp);
    }

    @Override
    public void setElevatorTarget(double meters, boolean goingUp) {
        setpointMeters = meters;
        isGoingUp = goingUp;
        double targetPosition = heightToTalonPos(Math.min(meters, MAX_EXTENSION_METERS.getValue()));

        // Apply the appropriate motion magic configs based on direction
        if (isGoingUp) {
            // Going up - use up configs for slower, more controlled movement
            motionRequest.Velocity = motionCruiseVelocityUp.getValue();
            motionRequest.Acceleration = motionAccelerationUp.getValue();
            motionRequest.Jerk = motionJerkUp.getValue();
        } else {
            // Going down - use down configs for faster movement (gravity assisted)
            motionRequest.Velocity = motionCruiseVelocityDown.getValue();
            motionRequest.Acceleration = motionAccelerationDown.getValue();
            motionRequest.Jerk = motionJerkDown.getValue();
        }

        leader.setControl(motionRequest.withPosition(targetPosition));
    }

    @Override
    public void resetElevatorPosition() {
        leader.setPosition(0.0);
        follower.setPosition(0.0);
    }

    @Override
    public double getElevatorVelocity() {
        return talonPosToHeight(leader.getVelocity().getValueAsDouble());
    }

    private double heightToTalonPos(double heightMeters) {
        return (heightMeters / (Math.PI * ELEVATOR_SPOOL_DIAMETER)) * ELEVATOR_GEAR_RATIO;
    }

    private double talonPosToHeight(double rotations) {
        return rotations * (Math.PI * ELEVATOR_SPOOL_DIAMETER) / ELEVATOR_GEAR_RATIO;
    }

    @Override
    public double getElevatorHeight() {
        return talonPosToHeight(leader.getPosition().getValueAsDouble());
    }
}
