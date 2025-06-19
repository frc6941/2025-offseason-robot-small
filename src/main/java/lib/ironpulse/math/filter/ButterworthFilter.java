package lib.ironpulse.math.filter;

public class ButterworthFilter implements DigitalFilter<Double> {
    private final double cutoffHz;
    private final double[] a = new double[3];
    private final double[] b = new double[3];
    private final double[] input = new double[3];
    private final double[] output = new double[3];

    public ButterworthFilter(double cutoffHz, double initVal, double dt_s) {
        this.cutoffHz = cutoffHz;
        reset(initVal);
        configureCoefficients(dt_s);
    }

    private void configureCoefficients(double dt_s) {
        double omega = Math.tan(Math.PI * cutoffHz * dt_s);
        double omega2 = omega * omega;
        double sqrt2 = Math.sqrt(2.0);
        double norm = 1.0 / (1.0 + sqrt2 * omega + omega2);

        b[0] = omega2 * norm;
        b[1] = 2.0 * b[0];
        b[2] = b[0];

        a[0] = 1.0;
        a[1] = 2.0 * (omega2 - 1.0) * norm;
        a[2] = (1.0 - sqrt2 * omega + omega2) * norm;
    }

    @Override
    public void input(Double value, double dt_s) {
        configureCoefficients(dt_s);
        input[0] = value;
        double result = b[0] * input[0] + b[1] * input[1] + b[2] * input[2]
                - a[1] * output[1] - a[2] * output[2];

        input[2] = input[1];
        input[1] = input[0];
        output[2] = output[1];
        output[1] = result;
        output[0] = result;
    }

    @Override
    public Double compute() {
        return output[0];
    }

    @Override
    public void reset(Double initVal) {
        for (int i = 0; i < 3; i++) {
            input[i] = initVal;
            output[i] = initVal;
        }
    }
}
