package frc.robot.Subsystems.EESubsystem;

public interface EEIO {
    // Set the speed of the actuator
    default void setSpeed(double speed) {}

    // Get the speed of the actuator
    default double getSpeed() { return 0.0; }

    // Stop the actuator
    default void stop() {}

    // Check if the actuator is currently moving
    default boolean isMoving() { return false; }

    // Get the current position of the actuator (actual position)
    default double getCurrentPosition() { return 0.0; }
}
