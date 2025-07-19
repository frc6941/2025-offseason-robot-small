package frc.robot.subsystems.elevator;

import edu.wpi.first.math.*;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Elevator.ELEVATOR_GEAR_RATIO;
import static frc.robot.Constants.Elevator.ELEVATOR_SPOOL_DIAMETER;
import static frc.robot.Constants.LOOPER_DT;
import static frc.robot.ElevatorCommonNT.ElevatorGainsClass;
import static frc.robot.ElevatorCommonNT.MAX_EXTENSION_METERS;


public class ElevatorIOSim implements ElevatorIO {
    public static final double carriaeMass = Units.lbsToKilograms(7.0 + (3.25 / 2));
    private static final DCMotor elevatorTalonsim = DCMotor.getKrakenX60Foc(2).withReduction(ELEVATOR_GEAR_RATIO);

    // Calculate scale factor from motor parameters
    private static final double SCALE_FACTOR = (elevatorTalonsim.KtNMPerAmp / ((ELEVATOR_SPOOL_DIAMETER / 2) * carriaeMass))
            * (1.0 / elevatorTalonsim.rOhms)
            * 0.5; // Damping factor to reduce response


    public static final Matrix<N2, N2> A =
            MatBuilder.fill(
                    Nat.N2(),
                    Nat.N2(),
                    0,
                    1,
                    0,
                    -elevatorTalonsim.KtNMPerAmp
                            / (elevatorTalonsim.rOhms
                            * Math.pow(ELEVATOR_SPOOL_DIAMETER / 2, 2)
                            * (carriaeMass)
                            * elevatorTalonsim.KvRadPerSecPerVolt));

    public static final Vector<N2> B =
            VecBuilder.fill(
                    0.0, elevatorTalonsim.KtNMPerAmp / ((ELEVATOR_SPOOL_DIAMETER / 2) * carriaeMass));
    private final ProfiledPIDController controller =
            new ProfiledPIDController(
                    ElevatorGainsClass.ELEVATOR_KP.getValue(),
                    ElevatorGainsClass.ELEVATOR_KI.getValue(),
                    ElevatorGainsClass.ELEVATOR_KD.getValue(),
                    new Constraints(
                            heightToRad(5.0),  // max velocity of 5 m/s
                            heightToRad(7.0)  // max acceleration of 10 m/s²
                    ));
    private Measure<VoltageUnit> appliedVolts = Volts.zero();
    private double targetPositionMeters = 0.0;
    private Vector<N2> simState;
    private double inputTorqueCurrent = 0.0;

    public ElevatorIOSim() {
        //initial position
        simState = VecBuilder.fill(0.0, 0.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        for (int i = 0; i < LOOPER_DT / (1.0 / 1000.0); i++) {
            // Calculate acceleration using state space model
            double acceleration = A.times(simState).get(1, 0) +
                    B.times(MatBuilder.fill(Nat.N1(), Nat.N1(), inputTorqueCurrent * SCALE_FACTOR)).get(1, 0) - 9.8;

            double feedforward = ElevatorGainsClass.ELEVATOR_KS.getValue() * Math.signum(simState.get(1)) +
                    ElevatorGainsClass.ELEVATOR_KV.getValue() * simState.get(1) +
                    ElevatorGainsClass.ELEVATOR_KA.getValue() * acceleration + // acceleration term
                    ElevatorGainsClass.ELEVATOR_KG.getValue(); // gravity compensation
            setInputTorqueCurrent(
                    controller.calculate(simState.get(0)) * SCALE_FACTOR + feedforward);
            update(1.0 / 1000.0);
        }

        inputs.positionMeters = radToHeight(simState.get(0));
        inputs.velocityMetersPerSec = radToHeight(simState.get(1));
        inputs.setpointMeters = targetPositionMeters;
        inputs.statorCurrentAmps = Math.copySign(inputTorqueCurrent, appliedVolts.magnitude());
        inputs.supplyCurrentAmps = Math.copySign(inputTorqueCurrent, appliedVolts.magnitude());
        inputs.motorVoltage = appliedVolts.magnitude();
    }

    private void update(double dt) {
        inputTorqueCurrent =
                MathUtil.clamp(inputTorqueCurrent, -elevatorTalonsim.stallCurrentAmps, elevatorTalonsim.stallCurrentAmps);
        Matrix<N2, N1> updatedState =
                NumericalIntegration.rkdp(
                        (Matrix<N2, N1> x, Matrix<N1, N1> u) ->
                                A.times(x)
                                        .plus(B.times(u))
                                        .plus(
                                                VecBuilder.fill(
                                                        0.0,
                                                        -9.8)),
                        simState,
                        MatBuilder.fill(Nat.N1(), Nat.N1(), inputTorqueCurrent * SCALE_FACTOR),
                        dt);
        // Apply limits
        simState = VecBuilder.fill(updatedState.get(0, 0), updatedState.get(1, 0));
        if (simState.get(0) <= 0.0) {
            simState.set(1, 0, 0.0);
            simState.set(0, 0, 0.0);
        }
        if (simState.get(0) >= heightToRad(MAX_EXTENSION_METERS.getValue())) {
            simState.set(1, 0, 0.0);
            simState.set(0, 0, heightToRad(MAX_EXTENSION_METERS.getValue()));
        }
    }

    private void setInputTorqueCurrent(double torqueCurrent) {
        inputTorqueCurrent = torqueCurrent;
        appliedVolts =
                Volts.of(elevatorTalonsim.getVoltage(
                        elevatorTalonsim.getTorque(inputTorqueCurrent),
                        simState.get(1, 0)));
        appliedVolts = Volts.of(MathUtil.clamp(appliedVolts.magnitude(), -12.0, 12.0));
    }

    @Override
    public void setElevatorVoltage(double volts) {
        setInputTorqueCurrent(volts / 12.0 * elevatorTalonsim.stallCurrentAmps);
    }

    @Override
    public void resetElevatorPosition() {
        simState.set(0, 0, 0.0);
    }

    @Override
    public void setElevatorTarget(double meters) {
        targetPositionMeters = meters;
        controller.setGoal(heightToRad(meters));
    }

    @Override
    public double getElevatorVelocity() {
        return radToHeight(simState.get(1));
    }

    private double heightToRad(double heightMeters) {
        return (heightMeters / (Math.PI * ELEVATOR_SPOOL_DIAMETER)) * ELEVATOR_GEAR_RATIO;
    }

    private double radToHeight(double rad) {
        return rad * (Math.PI * ELEVATOR_SPOOL_DIAMETER) / ELEVATOR_GEAR_RATIO;
    }

    @Override
    public double getElevatorHeight() {
        return radToHeight(simState.get(0));
    }
}