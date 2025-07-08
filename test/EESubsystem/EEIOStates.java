package frc.robot.Subsystems.EESubsystem;

public enum EEIOStates {
    IDLEORHOLD(0.0),
    INTAKE(500),
    SHOOT(1000);
    // Add more states as needed
    
    double expectedVelocity;

    EEIOStates(double expectedVelocity) {
        this.expectedVelocity = expectedVelocity;
    }


}
