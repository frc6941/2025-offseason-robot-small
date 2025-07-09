package frc.robot.drivers.led.patterns;
 
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
 
import frc.robot.drivers.led.AddressableLEDPattern;
 
// Represents a pattern where all LEDs are set to a solid color
public class SolidColorPattern implements AddressableLEDPattern {
 private final Color color;
 
 // Constructor to initialize the pattern with a specific color
 public SolidColorPattern(Color color) {
  this.color = color;
 }
 
 // Sets all LEDs in the buffer to the specified solid color
 @Override
 public void setLEDs(AddressableLEDBuffer buffer) {
  for (int index = 0; index < buffer.getLength(); index++) {
   buffer.setLED(index, color);
  }
 }
}