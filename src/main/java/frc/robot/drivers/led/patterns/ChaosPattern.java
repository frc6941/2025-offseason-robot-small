package frc.robot.drivers.led.patterns;
 
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
 
import frc.robot.drivers.led.AddressableLEDPattern;
 
// Represents a chaotic LED pattern where colors of the LEDs are randomly shifted over time
public class ChaosPattern implements AddressableLEDPattern {
 private boolean firstTime = true;
 
 // Sets the LEDs to a random color initially and then shifts their colors randomly in subsequent calls
 @Override
 public void setLEDs(AddressableLEDBuffer buffer) {
  if (firstTime) {
   for (var index = 0; index < buffer.getLength(); index++) {
    buffer.setLED(index,
      new Color(Math.random(), Math.random(), Math.random()));
   }
   firstTime = false;
  }
  for (var index = 0; index < buffer.getLength(); index++) {
   buffer.setLED(index, randomColorShift(buffer.getLED(index)));
  }
 }
 
 // Generates a new color by randomly shifting each component of the given color
 private Color randomColorShift(Color color) {
  return new Color(
    randomShift(color.red),
    randomShift(color.green),
    randomShift(color.blue));
 }
 
 // Shifts a color component value by a random amount in either direction, ensuring the value remains between 0 and 1
 private double randomShift(double value) {
  var sign = Math.random() >= 0.5 ? 1.0 : -1.0;
  var amount = Math.random() / 10;
  return MathUtil.clamp(value + sign * amount, 0, 1);
 }
 
 // Indicates that this pattern is animated and changes over time
 @Override
 public boolean isAnimated() {
  return true;
 }
}