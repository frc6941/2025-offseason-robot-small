package frc.robot.drivers.led.patterns;
 
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
 
import frc.robot.drivers.led.AddressableLEDPattern;
 
// Represents a pattern that alternates between a set of colors along an Addressable LED buffer
public class AlternatingColorPattern implements AddressableLEDPattern {
 private final Color[] colors;
 
 // Constructor to initialize the pattern with an array of colors
 public AlternatingColorPattern(Color[] colors) {
  this.colors = colors;
 }
 
 // Sets the LEDs in the buffer to create an alternating color pattern
 @Override
 public void setLEDs(AddressableLEDBuffer buffer) {
  for (var index = 0; index < buffer.getLength(); index++) {
   buffer.setLED(index, colors[index % colors.length]);
  }
 }
}