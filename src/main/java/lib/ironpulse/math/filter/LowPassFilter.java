package lib.ironpulse.math.filter;

public class LowPassFilter implements DigitalFilter<Double> {
    private final double cutoffHz;
    private double alpha;
    private double state;
    private double currentInput;

    public LowPassFilter(double cutoffHz, double initVal) {
        this.cutoffHz = cutoffHz;
        this.state = initVal;
    }

    @Override
    public void input(Double value, double dt_s) {
        this.currentInput = value;
        double tau = 1.0 / (2 * Math.PI * cutoffHz);
        this.alpha = dt_s / (tau + dt_s);
        state += alpha * (currentInput - state);
    }

    @Override
    public Double compute() {
        return state;
    }

    @Override
    public void reset(Double initVal) {
        this.state = initVal;
    }
}
