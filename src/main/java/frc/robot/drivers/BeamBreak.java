package frc.robot.drivers;

import edu.wpi.first.wpilibj.AnalogInput;

public class BeamBreak {
    // Represents the analog input channel used for the beam break sensor
    private final AnalogInput analogInput;
    // Stores the last status of the beam break sensor
    private boolean lastStatus;
    // Indicates if the beam break sensor was recently tripped
    private boolean tripped;
    // Indicates if the beam break sensor was recently cleared
    private boolean cleared;

    // Constructor initializes the AnalogInput with the specified channel
    public BeamBreak(int channel) {
        analogInput = new AnalogInput(channel);
    }

    public double getVoltage() {
        return analogInput.getVoltage();
    }

    // Updates the status of the beam break sensor and sets tripped or cleared flags accordingly
    public void update() {
        boolean value = get();
        tripped = value && !lastStatus;
        cleared = !value && lastStatus;
        lastStatus = value;
    }

    // Reads the current status of the beam break sensor based on its voltage level
    public boolean get() {
        return analogInput.getVoltage() > 3;
    }

    // Returns true if the beam break sensor was tripped since the last update
    public boolean wasTripped() {
        return tripped;
    }

    // Returns true if the beam break sensor was cleared since the last update
    public boolean wasCleared() {
        return cleared;
    }
}