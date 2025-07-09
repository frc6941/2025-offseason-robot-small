package frc.robot.drivers.led;
 
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
 
// Interface defining a pattern for Addressable LEDs
public interface AddressableLEDPattern {
    // Method to set LEDs based on the defined pattern
    void setLEDs(AddressableLEDBuffer buffer);
 
    // Default method to check if the pattern is animated
    // Returns false by default, indicating the pattern is static
    default boolean isAnimated() {
        return false;
    }
}