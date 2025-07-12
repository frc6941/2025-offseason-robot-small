package lib.ironpulse.swerve;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import frc.robot.SwerveModuleParamsNT;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static lib.ironpulse.math.MathTools.unwrapAngle;

public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged data;
    private final SwerveConfig swerveConfig;
    private final SwerveConfig.SwerveModuleConfig moduleConfig;
    @Getter
    private SwerveModulePosition[] odometryPositions;

    public SwerveModule(SwerveConfig swerveConfig, SwerveConfig.SwerveModuleConfig moduleConfig, SwerveModuleIO io) {
        // initialize
        this.io = io;
        this.swerveConfig = swerveConfig;
        this.moduleConfig = moduleConfig;
        this.data = new SwerveModuleIOInputsAutoLogged();
    }

    /**
     * Helper method to update drive controller parameters (PID + FF) by reading current TunableNumber values
     */
    private void updateDriveController() {
        double kp = SwerveModuleParamsNT.Drive.kP.getValue();
        double ki = SwerveModuleParamsNT.Drive.kI.getValue();
        double kd = SwerveModuleParamsNT.Drive.kD.getValue();
        double ks = SwerveModuleParamsNT.Drive.kS.getValue();
        double kv = SwerveModuleParamsNT.Drive.kV.getValue();
        double ka = SwerveModuleParamsNT.Drive.kA.getValue();
        io.configDriveController(kp, ki, kd, ks, kv, ka);
    }

    /**
     * Helper method to update steer controller parameters (PID + static friction) by reading current TunableNumber values
     */
    private void updateSteerController() {
        double kp = SwerveModuleParamsNT.Steer.kP.getValue();
        double ki = SwerveModuleParamsNT.Steer.kI.getValue();
        double kd = SwerveModuleParamsNT.Steer.kD.getValue();
        double ks = SwerveModuleParamsNT.Steer.kS.getValue();
        io.configSteerController(kp, ki, kd, ks);
    }

    public void updateInputs() {
        io.updateInputs(data);
        Logger.processInputs(swerveConfig.name + "/Module/" + moduleConfig.name, data);
    }

    public void periodic() {
        // compute position for odometry
        int sampleCount = Math.min(data.driveMotorPositionRadSamples.length, data.steerMotorPositionRadSamples.length);
        odometryPositions = new SwerveModulePosition[sampleCount];

        for (int i = 0; i < sampleCount; i++)
            odometryPositions[i] = new SwerveModulePosition(
                    data.driveMotorPositionRadSamples[i] * swerveConfig.wheelDiameter.in(Meter) * 0.5,
                    new Rotation2d(data.steerMotorPositionRadSamples[i])
            );

        // run dynamic parameter updates
        if (SwerveModuleParamsNT.Drive.isAnyChanged())
            updateDriveController();
        if (SwerveModuleParamsNT.Steer.isAnyChanged())
            updateSteerController();
    }

    public void runState(SwerveModuleState state) {
        io.setDriveVelocity(MetersPerSecond.of(state.speedMetersPerSecond));
        io.setSteerAngleAbsolute(state.angle.getMeasure());
    }

    public void runState(SwerveModuleState state, Current ff) {
        io.setDriveVelocity(MetersPerSecond.of(state.speedMetersPerSecond), ff);
        io.setSteerAngleAbsolute(state.angle.getMeasure());
    }

    public void runDriveVoltage(Voltage voltage) {
        io.setDriveOpenLoop(voltage);
        io.setSteerAngleAbsolute(Rotation2d.kZero.getMeasure());
    }

    public void runStop() {
        io.setDriveVelocity(MetersPerSecond.zero());
        io.setSteerOpenLoop(Volts.zero());
    }

    public Distance getDriveDistance() {
        return swerveConfig.wheelDiameter.times(data.driveMotorPositionRad * 0.5);
    }

    public LinearVelocity getDriveVelocity() {
        return MetersPerSecond.of(data.driveMotorVelocityRadPerSec * 0.5 * swerveConfig.wheelDiameter.in(Meter));
    }

    public Angle getSteerAngle() {
        return Radian.of(data.steerMotorPositionRad);
    }

    public AngularVelocity getSteerAngularVelocity() {
        return RadiansPerSecond.of(data.steerMotorVelocityRadPerSec);
    }


    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(unwrapAngle(0.0, getSteerAngle().in(Radian))));
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDriveDistance(), new Rotation2d(getSteerAngle()));
    }

    public SwerveModulePosition[] getSampledSwerveModulePositions() {
        int sampleCount = Math.min(data.driveMotorPositionRadSamples.length, data.steerMotorPositionRadSamples.length);
        SwerveModulePosition[] positions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++)
            positions[i] = new SwerveModulePosition(
                    swerveConfig.wheelDiameter.times(data.driveMotorPositionRadSamples[i] * 0.5),
                    new Rotation2d(data.steerMotorPositionRadSamples[i])
            );
        return positions;
    }
}
