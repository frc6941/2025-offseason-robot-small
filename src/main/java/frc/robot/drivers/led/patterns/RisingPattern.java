package frc.robot.drivers.led.patterns;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.drivers.led.AddressableLEDPattern;

public class RisingPattern implements AddressableLEDPattern {
    private final LEDPattern pattern;
    private final long periodMicros;

    public RisingPattern(Color color, Time period) {
        pattern = LEDPattern.solid(color).mask(LEDPattern.progressMaskLayer(this::getDim));
        periodMicros = (long) period.in(Units.Microseconds);
    }

    public double getDim() {
        double t = (double) (RobotController.getTime() % periodMicros) / periodMicros;
        double phase = t * 2.0 * Math.PI;
        return (Math.cos(phase) + 1.0) / 2.0;
    }

    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        pattern.applyTo(buffer);
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}