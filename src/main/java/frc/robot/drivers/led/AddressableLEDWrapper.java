package frc.robot.drivers.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.drivers.led.patterns.SolidColorPattern;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

@SuppressWarnings("PMD")
public class AddressableLEDWrapper {
    // Represents the Addressable LED strip
    private final AddressableLED addressableLED;
    // Buffer to hold the LED data
    private final AddressableLEDBuffer buffer;
    // LoggedDashboardString to log the first LED's color to NetworkTables for debugging
    @SuppressWarnings("deprecation")
    private final LoggedDashboardString dashboardBuffer = new LoggedDashboardString("Buffer");
    // Current pattern applied to the LED strip
    @Setter
    @Getter
    private AddressableLEDPattern pattern = new SolidColorPattern(Color.kBlack);
    // Intensity level for the LED strip, values range from 0.0 to 1.0
    @Setter
    private double intensity = 0.1;
    // Notifier to periodically update the LED strip
    private final Notifier looper = new Notifier(this::update);
    // Period in seconds at which the LED strip updates
    private double period = 0.05;

    // Constructor to initialize the AddressableLEDWrapper with the specified port and length
    public AddressableLEDWrapper(int port, int length) {
        addressableLED = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);
        addressableLED.setLength(length);
        addressableLED.setData(buffer);
        addressableLED.start();
        start(period);
    }

    // Sets the update period for the LED strip and restarts the looper
    public void setPeriod(double period) {
        this.period = period;
        stop();
        start(period);
    }

    // Starts the looper with the specified update period
    public void start(double period) {
        looper.startPeriodic(period);
    }

    // Updates the LED strip with the current pattern and adjusts the intensity of the LEDs
    public void update() {
        pattern.setLEDs(buffer);
        for (int i = 0; i < buffer.getLength(); i++) {
            var originalColor = buffer.getLED(i);
            var intensityAdjustedColor = new Color(
                    originalColor.red * intensity,
                    originalColor.green * intensity,
                    originalColor.blue * intensity);
            buffer.setLED(i, intensityAdjustedColor);
            dashboardBuffer.set(buffer.getLED(0).toHexString());
        }
        addressableLED.setData(buffer);
    }

    // Stops the looper, effectively halting updates to the LED strip
    public void stop() {
        looper.stop();
    }
}