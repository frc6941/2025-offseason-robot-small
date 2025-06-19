package lib.ironpulse.math.filter;

/**
 * Generic interface for digital filters.
 *
 * @param <T> type of data.
 */
public interface DigitalFilter<T> {
    /**
     * Input value, i.e. x[k].
     *
     * @param value
     * @param dt_s
     */
    void input(T value, double dt_s);

    /**
     * Computes the next output value from the filter, i.e. y[k].
     *
     * @return the filtered output
     */
    T compute();

    /**
     * Resets the filter.
     */
    void reset(T initVal);
}
