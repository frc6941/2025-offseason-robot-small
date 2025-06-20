package lib.ironpulse.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    // read
    default void updateInputs(SwerveModuleIOInputs inputs) {
    }

    // set
    default void setSwerveModuleState(SwerveModuleState state) {
    }
    default void setDriveOpenLoop(Voltage des) {
    }
    default void setDriveVelocity(LinearVelocity des) {
    }
    default void setDriveVelocity(LinearVelocity des, Current ff) {
    }
    default void setSteerOpenLoop(Voltage des) {
    }
    default void setSteerAngleAbsolute(Angle des) {
    }

    // config
    default void configDriveKp(double kp) {
    }
    default void configDriveKi(double ki) {
    }
    default void configDriveKd(double kd) {
    }
    default void configDriveFF(SimpleMotorFeedforward ff) {
    }
    default void configDriveBrake(boolean isBreak) {
    }
    default void configSteerKp(double kp) {
    }
    default void configSteerKi(double ki) {
    }
    default void configSteerKd(double kd) {
    }
    default void configSteerBrake(boolean isBreak) {
    }


    @AutoLog
    class SwerveModuleIOInputs {
        public boolean driveMotorConnected;
        public double driveMotorPositionRad;
        public double[] driveMotorPositionRadSamples;
        public double driveMotorVelocityRadPerSec;
        public double driveMotorTemperatureCel;
        public double driveMotorVoltageVolt;
        public double driveMotorSupplyCurrentAmpere;
        public double driveMotorTorqueCurrentAmpere;

        public boolean steerMotorConnected;
        public double steerMotorPositionRad;
        public double[] steerMotorPositionRadSamples;
        public double steerMotorVelocityRadPerSec;
        public double steerMotorTemperatureCel;
        public double steerMotorVoltageVolt;
        public double steerMotorSupplyCurrentAmpere;
        public double steerMotorTorqueCurrentAmpere;
    }
}
