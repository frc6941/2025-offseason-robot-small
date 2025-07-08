package lib.frcteam1678.io;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.frcteam1678.io.LightsIO.State.RGBColor;

public class LightsIOSim extends LightsIO {
	private final RGBColor[] leds;

	public LightsIOSim(int numLeds) {
		super(numLeds);
		leds = new RGBColor[numLeds];
		for (RGBColor led : leds) {
			led = new RGBColor(0, 0, 0);
		}
	}

	@Override
	protected void setLEDs(RGBColor color, int startIndex, int numLeds) {
		for (int i = startIndex; i < startIndex + numLeds; i++) {
			leds[i] = color;
			SmartDashboard.putNumberArray("LEDs Sim/" + i, new double[] {leds[i].r, leds[i].g, leds[i].b});
		}
	}
}
