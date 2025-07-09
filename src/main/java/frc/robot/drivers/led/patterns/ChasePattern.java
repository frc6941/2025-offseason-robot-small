package frc.robot.drivers.led.patterns;
 
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
 
import frc.robot.drivers.led.AddressableLEDPattern;
 
// Represents a chase pattern for an addressable LED strip
public class ChasePattern implements AddressableLEDPattern {
 private final Color[] colors; // Array of colors to use in the chase pattern
 private final int segmentWidth; // Width of each color segment in the pattern
 private int offset; // Current offset for the chase effect
 
 // Constructor to initialize the chase pattern with specified colors and segment width
 public ChasePattern(Color[] colors, int segmentWidth) {
  this.colors = colors;
  this.segmentWidth = segmentWidth;
 }
 
 // Sets the LEDs in the buffer to create a chase pattern effect
 @Override
 public void setLEDs(AddressableLEDBuffer buffer) {
  int effectiveIndex; // Effective index in the buffer after applying the offset
  int colorIndex; // Index of the color to set for the current segment
  int bufferLength = buffer.getLength(); // Length of the LED buffer
  for (var index = 0; index < bufferLength; index++) {
   effectiveIndex = (index + offset) % bufferLength;
   colorIndex = (index / segmentWidth) % colors.length;
   buffer.setLED(effectiveIndex, colors[colorIndex]);
  }
 
  offset = (offset + 1) % bufferLength; // Update the offset for the next frame of the chase pattern
 }
 
 // Indicates that this pattern is animated
 @Override
 public boolean isAnimated() {
  return true;
 }
}