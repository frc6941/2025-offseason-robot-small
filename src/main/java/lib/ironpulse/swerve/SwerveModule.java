package lib.ironpulse.swerve;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import frc.robot.SwerveModuleParamsNT;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Swerve.kSwerveModuleTag;

public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged data;
    private final SwerveConfig swerveConfig;
    private final SwerveConfig.SwerveModuleConfig moduleConfig;
    @Getter
    private SwerveModulePosition[] odometryPositions;

    public SwerveModule(SwerveConfig swerveConfig, SwerveConfig.SwerveModuleConfig moduleConfig, SwerveModuleIO io) {
        this.io = io;
        this.swerveConfig = swerveConfig;
        this.moduleConfig = moduleConfig;
        this.data = new SwerveModuleIOInputsAutoLogged();
    }

    public void updateInputs() {
        io.updateInputs(data);
        Logger.processInputs(kSwerveModuleTag + "/" + moduleConfig.name, data);
    }

    public void periodic() {
        // config io
//        io.configDriveKp(SwerveModuleParamsNT.driveKp());
//        io.configDriveKi(SwerveModuleParamsNT.driveKi());
//        io.configDriveKd(SwerveModuleParamsNT.driveKd());
//        io.configDriveFF(new SimpleMotorFeedforward(
//                SwerveModuleParamsNT.driveKs(), SwerveModuleParamsNT.driveKv(),
//                                                    SwerveModuleParamsNT.driveKa()
//        ));
//        io.configDriveBrake(SwerveModuleParamsNT.driveIsBrake());
//
//        io.configSteerKp(SwerveModuleParamsNT.steerKp());
//        io.configSteerKi(SwerveModuleParamsNT.steerKi());
//        io.configSteerKd(SwerveModuleParamsNT.steerKd());
//        io.configSteerBrake(SwerveModuleParamsNT.steerIsBrake());

        // compute position for odometry
//        int sampleCount = data.driveMotorPositionRadSamples.length;
//        odometryPositions = new SwerveModulePosition[sampleCount];
//
//        for (int i = 0; i < data.driveMotorPositionRadSamples.length; i++)
//            odometryPositions[i] = new SwerveModulePosition(
//                    data.driveMotorPositionRadSamples[i] * swerveConfig.wheelDiameter.in(Meter) * 0.5,
//                    new Rotation2d(data.steerMotorPositionRadSamples[i])
//            );
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
        io.setDriveOpenLoop(Volts.zero());
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
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerAngle()));
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDriveDistance(), new Rotation2d(getSteerAngle()));
    }
}
