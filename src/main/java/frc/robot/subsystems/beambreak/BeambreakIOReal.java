package frc.robot.subsystems.beambreak;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.drivers.BeamBreak;

public class BeambreakIOReal implements BeambreakIO {
    final BeamBreak beambreak;
    private final LinearFilter voltageFilter = LinearFilter.movingAverage(5);
    private double voltageFilterValue = 0.0;

    public BeambreakIOReal(int id) {
        beambreak = new BeamBreak(id);
    }

    public void updateInputs(BeambreakIOInputs inputs) {
        inputs.isBeambreakOn = beambreak.get();
        voltageFilterValue = voltageFilter.calculate(beambreak.getVoltage());
        inputs.voltage = voltageFilterValue;
    }
}
