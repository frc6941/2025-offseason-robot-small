package lib.ironpulse.utils;

import lombok.Setter;

/**
 * EdgedBoolean is a wrapper around a boolean value that detects specific edge transitions.
 */
public class EdgedBoolean {
    @Setter
    private Edge edge;
    private boolean lastValue = false;
    private boolean seenRising = false;

    /**
     * Constructs an EdgedBoolean with a specified edge detection mode.
     *
     * @param edge the edge type to detect
     */
    public EdgedBoolean(Edge edge) {
        this.edge = edge;
    }

    /**
     * Updates the internal state and returns true if the configured edge is detected.
     *
     * @param value the current boolean input
     * @return true if the specified edge is detected
     */
    public boolean update(boolean value) {
        boolean result = false;

        switch (edge) {
            case RISING:
                result = !lastValue && value;
                break;
            case FALLING:
                result = lastValue && !value;
                break;
            case RISING_THEN_FALLING:
                if (!lastValue && value) {
                    seenRising = true;
                } else if (seenRising && lastValue && !value) {
                    seenRising = false;
                    result = true;
                }
                break;
        }

        lastValue = value;
        return result;
    }

    /**
     * Resets the internal state.
     */
    public void reset() {
        lastValue = false;
        seenRising = false;
    }

    public enum Edge {
        RISING,
        FALLING,
        RISING_THEN_FALLING
    }
}
