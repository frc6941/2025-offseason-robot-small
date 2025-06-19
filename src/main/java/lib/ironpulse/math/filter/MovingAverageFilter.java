package lib.ironpulse.math.filter;

import java.util.LinkedList;
import java.util.Queue;

public class MovingAverageFilter implements DigitalFilter<Double> {
    private final double cutoffHz;
    private final Queue<Double> window = new LinkedList<>();
    private int windowSize = 1;
    private double sum = 0;

    public MovingAverageFilter(double cutoffHz, double initVal) {
        this.cutoffHz = cutoffHz;
        reset(initVal);
    }

    @Override
    public void input(Double value, double dt_s) {
        // Update window size based on frequency
        windowSize = Math.max(1, (int) Math.round(1.0 / (cutoffHz * dt_s)));
        if (window.size() == windowSize) {
            sum -= window.poll();
        }
        window.add(value);
        sum += value;
    }

    @Override
    public Double compute() {
        return sum / window.size();
    }

    @Override
    public void reset(Double initVal) {
        window.clear();
        sum = 0;
        for (int i = 0; i < windowSize; i++) {
            window.add(initVal);
            sum += initVal;
        }
    }
}
