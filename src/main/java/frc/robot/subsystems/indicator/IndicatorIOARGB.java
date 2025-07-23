package frc.robot.subsystems.indicator;

import frc.robot.drivers.led.AddressableLEDWrapper;
import frc.robot.drivers.led.patterns.RainbowingPattern;

import static frc.robot.Constants.Indicator.LED_BUFFER_LENGTH;
import static frc.robot.Constants.Indicator.LED_PORT;

public class IndicatorIOARGB implements IndicatorIO {
    private final AddressableLEDWrapper led;
    private Patterns currentPattern = Patterns.AUTO;

    public IndicatorIOARGB() {
        led = new AddressableLEDWrapper(
                LED_PORT,
                LED_BUFFER_LENGTH
        );
        led.setIntensity(1);
        led.start(0.05);
    }

    @Override
    public void updateInputs(IndicatorIOInputs inputs) {
        setPattern(currentPattern);
        inputs.currentPattern = currentPattern;
    }

    @Override
    public void setPattern(Patterns pattern) {
        currentPattern = pattern;
        if (pattern == Patterns.AUTO) {
            led.setPattern(new RainbowingPattern());
            return;
        }
        led.setPattern(pattern.pattern);
    }

    @Override
    public void reset() {
        led.stop();
        led.start(0.02);
    }
}
