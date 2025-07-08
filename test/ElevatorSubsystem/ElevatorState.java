package frc.robot.Subsystems.ElevatorSubsystem;

public enum ElevatorState {
    L1(0.0),
    L2(0.25),
    L3(0.5),
    L4(1);
    // Add more states as needed
    
    double expectedPosition;

    ElevatorState(double expectedPosition) {
        this.expectedPosition = expectedPosition;
    }


}
