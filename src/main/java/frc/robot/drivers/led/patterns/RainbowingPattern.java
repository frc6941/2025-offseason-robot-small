package frc.robot.drivers.led.patterns;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.drivers.led.AddressableLEDPattern;

public class RainbowingPattern implements AddressableLEDPattern {
    LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    LEDPattern scrollingRainbow =
            rainbow.scrollAtAbsoluteSpeed(
                    Units.MetersPerSecond.of(0.1),
                    Units.Meters.of((double) 1 / 65));

    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        scrollingRainbow.applyTo(buffer);
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}