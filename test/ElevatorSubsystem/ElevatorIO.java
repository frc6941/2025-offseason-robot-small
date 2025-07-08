package frc.robot.Subsystems.ElevatorSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    default void updateInputs(ElevatorIOInputs inputs){}

    default void setElevatorVoltage(double volts) {
    }

    default void setElevatorTarget(double meters) {
    }

    default void resetElevatorPosition() {
    }

    default double getElevatorVelocity() {
        return 0.0;
    }

    default void configureSlot0Config(double kp, double ki, double kd, double ka, double kv, double ks, double kg){}

    @AutoLog
    public static class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double setpointMeters = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double motorVoltage = 0.0;
        public double tempCelsius = 0.0;
    }
}
