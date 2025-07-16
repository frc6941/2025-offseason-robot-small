package lib.ironpulse.swerve;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
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
        setDriveVelocity(des);
    }
    default void setSteerOpenLoop(Voltage des) {
    }
    default void setSteerAngleAbsolute(Angle des) {
    }

    // config
    default void configDriveController(double kp, double ki, double kd, double ks, double kv, double ka) {
    }
    default void configDriveBrake(boolean isBreak) {
    }
    default void configSteerController(double kp, double ki, double kd, double ks) {
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
