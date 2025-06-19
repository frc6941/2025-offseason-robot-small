package lib.ironpulse.math.filter;

public class LeakyIntegratorFilter implements DigitalFilter<Double> {
    private final double cutoffHz;
    private double leakRate;
    private double state;
    private double input;

    public LeakyIntegratorFilter(double cutoffHz, double initVal) {
        this.cutoffHz = cutoffHz;
        this.state = initVal;
    }

    @Override
    public void input(Double value, double dt_s) {
        this.input = value;
        double tau = 1.0 / (2 * Math.PI * cutoffHz);
        this.leakRate = dt_s / (tau + dt_s);
        state = (1.0 - leakRate) * state + leakRate * input;
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
