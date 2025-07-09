package frc.robot.drivers.led.patterns;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import lombok.Setter;

import frc.robot.drivers.led.AddressableLEDPattern;

// Represents a pattern where the color intensity is interpolated between a low and high color
public class IntensityPattern implements AddressableLEDPattern {
    private final Color highColor;
    private final Color lowColor;

    // Sets the intensity of the color interpolation, ranging from 0 (lowColor) to 1 (highColor)
    @Setter
    private double intensity;

    /**
     * Constructs an IntensityPattern with a default low color of black and a specified high color and intensity
     * @param highColor Brightest color
     * @param intensity 0..1 with 1 being the color and 0 being black
     */
    public IntensityPattern(Color highColor, double intensity) {
        this(Color.kBlack, highColor, intensity);
    }

    // Constructs an IntensityPattern with specified low and high colors and intensity
    public IntensityPattern(Color lowColor, Color highColor, double intensity) {
        this.highColor = highColor;
        this.lowColor = lowColor;
        this.intensity = intensity;
    }

    // Sets the LEDs in the buffer to a color interpolated between lowColor and highColor based on intensity
    @Override
    public void setLEDs(AddressableLEDBuffer buffer) {
        for (var index = 0; index < buffer.getLength(); index++) {
            buffer.setLED(index, new Color(
                    MathUtil.interpolate(lowColor.red, highColor.red, intensity),
                    MathUtil.interpolate(lowColor.green, highColor.green, intensity),
                    MathUtil.interpolate(lowColor.blue, highColor.blue, intensity)));
        }
    }

    // Indicates whether this pattern is animated
    @Override
    public boolean isAnimated() {
        return true;
    }
}