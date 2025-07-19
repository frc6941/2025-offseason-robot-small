package lib.ironpulse.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import lombok.Builder;
import lombok.Getter;
import lombok.Setter;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static edu.wpi.first.units.Units.*;
import static lib.ironpulse.math.MathTools.*;

/**
 * "Inspired" by FRC team 254. See the license file in the root directory of this project.
 *
 * <p>
 * Takes a prior setpoint (ChassisSpeeds), a desired setpoint (from a driver, or from a path
 * follower), and outputs a new setpoint that respects all the kinematic constraints on module
 * rotation speed and wheel velocity/acceleration. By generating a new setpoint every iteration, the
 * robot will converge to the desired setpoint quickly while avoiding any intermediate state that is
 * kinematically infeasible (and can result in wheel slip or robot heading drift as a result).
 * </p>
 */
@Builder
public class SwerveSetpointGenerator {
    private final SwerveDriveKinematics kinematics;

    @Getter
    @Setter
    private SwerveLimit chassisLimit;
    @Getter
    @Setter
    private SwerveModuleLimit moduleLimit;


    public SwerveSetpoint generate(ChassisSpeeds desiredChassisSpeed, SwerveSetpoint prevSetpoint, double dt) {
        // apply limit at chassis-level
        desiredChassisSpeed = chassisLimit.apply(prevSetpoint.chassisSpeeds(), desiredChassisSpeed, dt);

        // compute module desired states
        SwerveModuleState[] desiredModuleState = kinematics.toSwerveModuleStates(desiredChassisSpeed);
        if (moduleLimit.maxDriveVelocity().magnitude() > 0.0) {
            SwerveDriveKinematics.desaturateWheelSpeeds(
                    desiredModuleState, moduleLimit.maxDriveVelocity().in(MetersPerSecond));
            desiredChassisSpeed = kinematics.toChassisSpeeds(desiredModuleState);
        }

        int n = kinematics.getModules().length;

        // Special case: desiredState is a complete stop.
        // In this case, module angle is arbitrary, so just use the previous angle.
        boolean needToSteer = true;
        if (epsilonEquals(toTwist2d(desiredChassisSpeed), new Twist2d(), 0.001)) {
            needToSteer = false;
            for (int i = 0; i < n; ++i) {
                desiredModuleState[i].angle = prevSetpoint.moduleStates()[i].angle;
                desiredModuleState[i].speedMetersPerSecond = 0.0;
            }
        }

        // compute vx and vy vectors.
        double[] vxPrev = new double[n];
        double[] vyPrev = new double[n];
        Rotation2d[] headingPrev = new Rotation2d[n];
        double[] vxDes = new double[n];
        double[] vyDes = new double[n];
        Rotation2d[] headingDes = new Rotation2d[n];
        boolean allModulesShouldFlip = true;

        for (int i = 0; i < kinematics.getModules().length; ++i) {
            vxPrev[i] = prevSetpoint.moduleStates()[i].angle.getCos() * prevSetpoint.moduleStates()[i].speedMetersPerSecond;
            vyPrev[i] = prevSetpoint.moduleStates()[i].angle.getSin() * prevSetpoint.moduleStates()[i].speedMetersPerSecond;
            headingPrev[i] = prevSetpoint.moduleStates()[i].angle;
            if (prevSetpoint.moduleStates()[i].speedMetersPerSecond < 0.0)
                headingPrev[i] = headingPrev[i].rotateBy(Rotation2d.fromRadians(Math.PI));

            vxDes[i] = desiredModuleState[i].angle.getCos() * desiredModuleState[i].speedMetersPerSecond;
            vyDes[i] = desiredModuleState[i].angle.getSin() * desiredModuleState[i].speedMetersPerSecond;
            headingDes[i] = desiredModuleState[i].angle;
            if (desiredModuleState[i].speedMetersPerSecond < 0.0)
                headingDes[i] = headingDes[i].rotateBy(Rotation2d.fromRadians(Math.PI));

            if (allModulesShouldFlip) {
                double requiredRotationRad = Math.abs(headingPrev[i].unaryMinus().rotateBy(headingDes[i]).getRadians());
                if (requiredRotationRad < Math.PI / 2.0) allModulesShouldFlip = false;
            }
        }

        if (allModulesShouldFlip
                && !epsilonEquals(toTwist2d(prevSetpoint.chassisSpeeds()), new Twist2d(), 0.001)
                && !epsilonEquals(toTwist2d(desiredChassisSpeed), new Twist2d(), 0.001))
            // It will (likely) be faster to stop the robot, rotate the modules in place to the complement of desired, and accelerate again.
            return generate(new ChassisSpeeds(), prevSetpoint, dt);


        // Compute the deltas between start and goal. We can then interpolate from the start state to the goal state; then find the
        // amount we can move from start towards goal in this cycle such that no kinematic limit is exceeded.
        double dx = desiredChassisSpeed.vxMetersPerSecond - prevSetpoint.chassisSpeeds().vxMetersPerSecond;
        double dy = desiredChassisSpeed.vyMetersPerSecond - prevSetpoint.chassisSpeeds().vyMetersPerSecond;
        double dtheta = desiredChassisSpeed.omegaRadiansPerSecond - prevSetpoint.chassisSpeeds().omegaRadiansPerSecond;

        // 's' interpolates between start and goal. At 0, we are at prevState and at 1, we are at
        // desiredState.
        double sMin = 1.0;

        // In cases where an individual module is stopped, we want to remember the right steering angle
        // to command (since inverse kinematics doesn't care about angle, we can be opportunistically lazy).
        List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(n);
        // Enforce steering velocity limits. We do this by taking the derivative of steering angle at the current angle, and then backing
        // out the maximum interpolant between start and goal states. We remember the  minimum across all modules, since that is the
        // active constraint.
        final double maxThetaStep = dt * moduleLimit.maxSteerAngularVelocity().in(RadiansPerSecond);
        for (int i = 0; i < n; ++i) {
            if (!needToSteer) {
                overrideSteering.add(Optional.of(prevSetpoint.moduleStates()[i].angle));
                continue;
            }
            overrideSteering.add(Optional.empty());

            if (epsilonEquals(prevSetpoint.moduleStates()[i].speedMetersPerSecond, 0.0, 0.001)) {
                // If module is stopped, we know that we will need to move straight to the final steering
                // angle, so limit based purely on rotation in place.
                if (epsilonEquals(desiredModuleState[i].speedMetersPerSecond, 0.0)) {
                    // Goal angle doesn't matter. Just leave module at its current angle.
                    overrideSteering.set(i, Optional.of(prevSetpoint.moduleStates()[i].angle));
                    continue;
                }

                var necessaryRotation = prevSetpoint.moduleStates()[i].angle.unaryMinus().rotateBy(
                        desiredModuleState[i].angle);
                if (shouldFlip(necessaryRotation))
                    necessaryRotation = necessaryRotation.rotateBy(Rotation2d.fromRadians(Math.PI));
                final double numStepsNeeded = Math.abs(necessaryRotation.getRadians()) / maxThetaStep;

                if (numStepsNeeded <= 1.0) {
                    // Steer directly to goal angle.
                    overrideSteering.set(i, Optional.of(desiredModuleState[i].angle));
                    // Don't limit the global min_s;
                    continue;
                } else {
                    // Adjust steering by max_theta_step.
                    overrideSteering.set(
                            i, Optional.of(prevSetpoint.moduleStates()[i].angle.rotateBy(
                                    Rotation2d.fromRadians(Math.signum(necessaryRotation.getRadians()) * maxThetaStep)))
                    );
                    sMin = 0.0;
                    continue;
                }
            }
            if (sMin == 0.0) {
                // s can't get any lower. Save some CPU.
                continue;
            }

            final int kMaxIterations = 8;
            double s = findSteerMaxS(
                    vxPrev[i], vyPrev[i], headingPrev[i].getRadians(), vxDes[i], vyDes[i], headingDes[i].getRadians(),
                    maxThetaStep, kMaxIterations
            );
            sMin = Math.min(sMin, s);
        }

        // Enforce drive wheel acceleration limits.
        final double max_vel_step = dt * moduleLimit.maxDriveAcceleration().in(MetersPerSecondPerSecond);
        for (int i = 0; i < n; ++i) {
            if (sMin == 0.0) {
                // No need to carry on.
                break;
            }
            double vx_min_s = sMin == 1.0 ? vxDes[i] : (vxDes[i] - vxPrev[i]) * sMin + vxPrev[i];
            double vy_min_s = sMin == 1.0 ? vyDes[i] : (vyDes[i] - vyPrev[i]) * sMin + vyPrev[i];
            // Find the max s for this drive wheel. Search on the interval between 0 and min_s, because we already know we can't go
            // faster than that.
            final int kMaxIterations = 10;
            double s = sMin * findDriveMaxS(
                    vxPrev[i], vyPrev[i], Math.hypot(vxPrev[i], vyPrev[i]), vx_min_s, vy_min_s,
                    Math.hypot(vx_min_s, vy_min_s), max_vel_step, kMaxIterations
            );
            sMin = Math.min(sMin, s);
        }

        ChassisSpeeds retSpeeds = new ChassisSpeeds(
                prevSetpoint.chassisSpeeds().vxMetersPerSecond + sMin * dx,
                prevSetpoint.chassisSpeeds().vyMetersPerSecond + sMin * dy,
                prevSetpoint.chassisSpeeds().omegaRadiansPerSecond + sMin * dtheta
        );
        var retStates = kinematics.toSwerveModuleStates(retSpeeds);
        for (int i = 0; i < n; ++i) {
            final var maybeOverride = overrideSteering.get(i);
            if (maybeOverride.isPresent()) {
                var override = maybeOverride.get();
                if (shouldFlip(retStates[i].angle.unaryMinus().rotateBy(override)))
                    retStates[i].speedMetersPerSecond *= -1.0;
                retStates[i].angle = override;
            }
            final var deltaRotation = prevSetpoint.moduleStates()[i].angle.unaryMinus().rotateBy(retStates[i].angle);
            if (shouldFlip(deltaRotation)) {
                retStates[i].angle = retStates[i].angle.rotateBy(Rotation2d.fromRadians(Math.PI));
                retStates[i].speedMetersPerSecond *= -1.0;
            }
        }
        return new SwerveSetpoint(retSpeeds, retStates);
    }

    /**
     * Check if it would be faster to go to the opposite of the goal heading while reverse drive direction.
     *
     * @param prevToGoal The rotation from the previous state to the goal state.
     * @return True if the shortest path to achieve this rotation involves flipping.
     */
    private boolean shouldFlip(Rotation2d prevToGoal) {
        return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
    }

    /**
     * Find the root of the generic 2D parametric function 'func' using the regula falsi technique.
     * This is a pretty naive way to do root finding, but it's usually faster than simple bisection
     * while being robust in ways that e.g. the Newton-Raphson method isn't.
     *
     * @param func           The {@link Function2d} to take the root of.
     * @param x0             x value of the lower bracket.
     * @param y0             y value of the lower bracket.
     * @param f0             value of 'func' at x_0, y_0 (passed in by caller to save a call to 'func' during recursion)
     * @param x1             x value of the upper bracket.
     * @param y1             y value of the upper bracket.
     * @param f1             value of 'func' at x_1, y_1 (passed in by caller to save a call to 'func' during recursion)
     * @param iterationsLeft Number of iterations of root finding left.
     * @return The parameter value 's' that interpolating between 0 and 1 that corresponds to the (approximate) root.
     */
    private double findRoot(Function2d func, double x0, double y0, double f0, double x1, double y1, double f1,
                            int iterationsLeft) {
        if (iterationsLeft <= 0 || epsilonEquals(f0, f1)) return 1.0;
        var sGuess = Math.max(0.0, Math.min(1.0, -f0 / (f1 - f0)));
        var xGuess = (x1 - x0) * sGuess + x0;
        var yGuess = (y1 - y0) * sGuess + y0;
        var fGuess = func.f(xGuess, yGuess);
        if (Math.signum(f0) == Math.signum(fGuess)) {
            // 0 and guess on same side of root, so use upper bracket.
            return sGuess + (1.0 - sGuess) * findRoot(func, xGuess, yGuess, fGuess, x1, y1, f1, iterationsLeft - 1);
        } else {
            // Use lower bracket.
            return sGuess * findRoot(func, x0, y0, f0, xGuess, yGuess, fGuess, iterationsLeft - 1);
        }
    }


    /**
     * Find drive max s.
     *
     * @param x0            x value of the lower bracket.
     * @param y0            y value of the lower bracket.
     * @param f0            value of 'func' at x_0, y_0.
     * @param x1            x value of the upper bracket.
     * @param y1            y value of the upper bracket.
     * @param f1            value of 'func' at x_1, y_1 (passed in by caller to save a call to 'func' during recursion)
     * @param maxVelStep    max vel difference from current.
     * @param maxIterations number of max iterations.
     * @return result s.
     */
    private double findDriveMaxS(double x0, double y0, double f0, double x1, double y1, double f1, double maxVelStep,
                                 int maxIterations) {
        double diff = f1 - f0;
        if (Math.abs(diff) < maxVelStep) return 1.0;
        double offset = f0 + Math.signum(diff) * maxVelStep;
        Function2d func = (x, y) -> Math.hypot(x, y) - offset;
        return findRoot(func, x0, y0, f0 - offset, x1, y1, f1 - offset, maxIterations);
    }

    /**
     * Find steer max s.
     *
     * @param x0            x value of the lower bracket.
     * @param y0            y value of the lower bracket.
     * @param f0            value of 'func' at x_0, y_0.
     * @param x1            x value of the upper bracket.
     * @param y1            y value of the upper bracket.
     * @param f1            value of 'func' at x_1, y_1 (passed in by caller to save a call to 'func' during recursion)
     * @param maxDeviation  max angle difference from current.
     * @param maxIterations number of max iterations.
     * @return result s.
     */
    private double findSteerMaxS(double x0, double y0, double f0, double x1, double y1, double f1, double maxDeviation,
                                 int maxIterations) {
        double diff = f1 - f0;
        if (Math.abs(diff) < maxDeviation) return 1.0;
        double offset = f0 + Math.signum(diff) * maxDeviation;
        Function2d func = (x, y) -> unwrapAngle(f0, Math.atan2(y, x)) - offset;
        return findRoot(func, x0, y0, f0 - offset, x1, y1, f1 - offset, maxIterations);
    }

    /**
     * Simple 2d Functional Interface.
     */
    @FunctionalInterface
    private interface Function2d {
        double f(double x, double y);
    }
}
