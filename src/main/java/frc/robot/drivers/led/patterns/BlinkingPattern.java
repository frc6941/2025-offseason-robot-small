package frc.robot.drivers.led.patterns;
 
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
 
import frc.robot.drivers.led.AddressableLEDPattern;
 
// Represents a blinking LED pattern using two solid color patterns and a specified interval
public class BlinkingPattern implements AddressableLEDPattern {
 private final AddressableLEDPattern onPattern;
 private final AddressableLEDPattern offPattern;
 private final double interval;
 private boolean on = true;
 private double lastChange;
 
 /**
  * Constructor for BlinkingPattern
  * @param onColor  color for when the blink is on.
  * @param interval time in seconds between changes.
  */
 public BlinkingPattern(Color onColor, double interval) {
  onPattern = new SolidColorPattern(onColor);
  offPattern = new SolidColorPattern(Color.kBlack);
  this.interval = interval;
 }
 
 // Updates the LED buffer to reflect the current state of the blinking pattern
 @Override
 public void setLEDs(AddressableLEDBuffer buffer) {
  var timestamp = Timer.getFPGATimestamp();
  if (timestamp - lastChange > interval) {
   on = !on;
   lastChange = timestamp;
  }
 
  if (on) {
   onPattern.setLEDs(buffer);
   return;
  }
  offPattern.setLEDs(buffer);
 }
 
 // Indicates that this pattern is animated and requires periodic updates
 @Override
 public boolean isAnimated() {
  return true;
 }
}