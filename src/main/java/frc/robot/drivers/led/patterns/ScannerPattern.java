package frc.robot.drivers.led.patterns;
 
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
 
import frc.robot.drivers.led.AddressableLEDPattern;
 
// Represents a scanner pattern for an addressable LED strip
public class ScannerPattern implements AddressableLEDPattern {
 private final Color eyeColor;
 private final Color backgroundColor;
 private final int length;
 private int eyePosition = 0;
 private int scanDirection = 1;
 
 // Constructs a scanner pattern with a specified eye color and default black background
 public ScannerPattern(Color eyeColor, int length) {
  this(eyeColor, Color.kBlack, length);
 }
 
 // Constructs a scanner pattern with specified eye color, background color, and length
 public ScannerPattern(Color eyeColor, Color backgroundColor, int length) {
  this.eyeColor = eyeColor;
  this.backgroundColor = backgroundColor;
  this.length = length;
 }
 
 // Updates the LED buffer to display the scanner pattern
 @Override
 public void setLEDs(AddressableLEDBuffer buffer) {
  var bufferLength = buffer.getLength();
  double intensity;
  double red;
  double green;
  double blue;
  double distanceFromEye;
 
  for (var index = 0; index < bufferLength; index++) {
   distanceFromEye = MathUtil.clamp(Math.abs(eyePosition - index), 0, length);
   intensity = 1 - distanceFromEye / length;
   red = MathUtil.interpolate(backgroundColor.red, eyeColor.red, intensity);
   green = MathUtil.interpolate(backgroundColor.green, eyeColor.green, intensity);
   blue = MathUtil.interpolate(backgroundColor.blue, eyeColor.blue, intensity);
 
   buffer.setLED(index, new Color(red, green, blue));
  }
 
  if (eyePosition == 0) {
   scanDirection = 1;
  } else if (eyePosition == bufferLength - 1) {
   scanDirection = -1;
  }
 
  eyePosition += scanDirection;
 }
 
 // Indicates that this pattern is animated
 @Override
 public boolean isAnimated() {
  return true;
 }
}