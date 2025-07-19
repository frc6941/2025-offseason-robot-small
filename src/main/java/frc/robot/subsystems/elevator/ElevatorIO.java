package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    default void updateInputs(ElevatorIOInputs inputs) {
    }

    default void setElevatorVoltage(double volts) {
    }

    default void setElevatorTarget(double meters) {
    }

    default void setElevatorTarget(double meters, boolean isGoingUp) {
        setElevatorTarget(meters); // Default fallback
    }

    default void resetElevatorPosition() {
    }

    default double getElevatorHeight() {
        return 0.0;
    }

    default double getElevatorVelocity() {
        return 0.0;
    }

    @AutoLog
    class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSec = 0.0;
        public double setpointMeters = 0.0;
        public double appliedVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double motorVoltage = 0.0;
        public double tempCelsius = 0.0;
        // Dynamic Motion Magic logging
        public boolean isGoingUp = false;
        public double currentAcceleration = 0.0;
        public double currentCruiseVelocity = 0.0;
        public double currentJerk = 0.0;
    }

}