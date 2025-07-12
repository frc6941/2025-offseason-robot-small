package lib.ironpulse.utils;

import edu.wpi.first.wpilibj.Timer;
import lombok.Setter;

/**
 * TimeDelayedBoolean is a wrapper around normal boolean with a timer.
 * TimeDelayedBoolean returns true whenever a value is hold true for more than a period of time specified.
 */
public class TimeDelayedBoolean {
    private final Timer timer;
    @Setter
    private double delaySeconds;

    /**
     * Constructor.
     *
     * @param delaySeconds required time period for value to be continuously true, in seconds.
     */
    public TimeDelayedBoolean(double delaySeconds) {
        this.delaySeconds = delaySeconds;
        timer = new Timer();
        timer.reset();
        timer.start();
    }

    /**
     * Force-reset the internal timer.
     */
    public void reset() {
        timer.reset();
    }

    /**
     * Core function for TimeDelayedBoolean.
     *
     * @param value current value.
     * @return value wrapped with delay requirements.
     */
    public boolean update(boolean value) {
        if (value) {
            return timer.hasElapsed(delaySeconds);
        }
        timer.reset();
        return false;
    }

    public boolean update(boolean value, double delaySeconds) {
        setDelaySeconds(delaySeconds);
        return update(value);
    }
}
