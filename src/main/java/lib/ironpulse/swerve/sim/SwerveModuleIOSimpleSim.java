package lib.ironpulse.swerve.sim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import lib.ironpulse.swerve.SwerveModuleIO;
import lib.ironpulse.utils.Logging;

import static edu.wpi.first.units.Units.*;

public class SwerveModuleIOSimpleSim implements SwerveModuleIO {
    private final SwerveSimConfig config;
    private final SwerveModuleState simState = new SwerveModuleState();
    private final SwerveModulePosition simPosition = new SwerveModulePosition();
    private double prevTimestamp;

    public SwerveModuleIOSimpleSim(SwerveSimConfig config, int idx) {
        this.config = config;
        // record initial time for simulation
        prevTimestamp = Timer.getTimestamp();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs data) {
        double now = Timer.getTimestamp();
        double dt = now - prevTimestamp;
        prevTimestamp = now;

        // integrate drive distance
        simPosition.distanceMeters += simState.speedMetersPerSecond * dt;
        // update module heading
        simPosition.angle = simState.angle;

        double driveVel = simState.speedMetersPerSecond;
        double steerVel = dt > 0
                ? (simPosition.angle.getRadians() - data.steerMotorPositionRad) / dt
                : 0.0;

        data.driveMotorConnected = true;
        data.driveMotorPositionRad = simPosition.distanceMeters * 2.0 / config.wheelDiameter.in(Meter);
        data.driveMotorPositionRadSamples = new double[]{data.driveMotorPositionRad};
        data.driveMotorVelocityRadPerSec = driveVel;

        data.steerMotorConnected = true;
        data.steerMotorPositionRad = simPosition.angle.getRadians();
        data.steerMotorPositionRadSamples = new double[]{data.steerMotorPositionRad};
        data.steerMotorVelocityRadPerSec = steerVel;
    }

    @Override
    public void setDriveOpenLoop(Voltage des) {
        simState.speedMetersPerSecond = des.in(Volts) / RobotController.getBatteryVoltage() * 4.0;
    }

    @Override
    public void setDriveVelocity(LinearVelocity linearVelocityDes) {
        simState.speedMetersPerSecond = linearVelocityDes.in(MetersPerSecond);
    }

    @Override
    public void setSteerAngleAbsolute(Angle des) {
        simState.angle = new Rotation2d(des);
        simPosition.angle = simState.angle;
    }

    @Override
    public void configDriveController(double kp, double ki, double kd, double ks, double kv, double ka) {
        Logging.info(
                config.name + "Module",
                "Module drive controller updated! kp: %.2f, ki: %.2f, kd: %.2f",
                kp, ki, kd
        );
    }

    @Override
    public void configSteerController(double kp, double ki, double kd, double ks) {
        Logging.info(
                config.name + "Module",
                "Module steer controller updated! kp: %.2f, ki: %.2f, kd: %.2f",
                kp, ki, kd
        );
    }
}
