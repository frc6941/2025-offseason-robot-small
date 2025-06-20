package lib.ironpulse.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import lib.ironpulse.math.MathTools;
import lombok.Builder;

import static edu.wpi.first.units.Units.*;

/**
 * Enforces velocity and acceleration at the chassis level.
 *
 * <p>
 * The {@code SwerveLimit} class defines maximum linear and angular speeds and accelerations for swerve chassis.
 * Use the {@link #apply(ChassisSpeeds, ChassisSpeeds, double)} method to clamp and integrate desired speeds based on the current state
 * and a time step.
 * </p>
 */
@Builder
public record SwerveLimit(LinearVelocity maxLinearVelocity, LinearAcceleration maxSkidAcceleration,
                          AngularVelocity maxAngularVelocity, AngularAcceleration maxAngularAcceleration) {
    /**
     * Apply linear and angular velocity and acceleration limits to the desired chassis speeds.
     *
     * @param curr current chassis speeds.
     * @param des  desired chassis speeds.
     * @param dt   delta time in seconds.
     * @return a new {@link ChassisSpeeds} instance with velocities limited and discretized.
     */
    public ChassisSpeeds apply(ChassisSpeeds curr, ChassisSpeeds des, double dt) {
        // go from continuous to discrete
        curr = ChassisSpeeds.discretize(curr, dt);

        // get limits in doubles
        double vMaxMps = maxLinearVelocity.in(MetersPerSecond);
        double aMaxMps2 = maxSkidAcceleration.in(MetersPerSecondPerSecond);
        double wMaxRadps = maxAngularVelocity.in(RadiansPerSecond);
        double alphaMaxRadps2 = maxAngularAcceleration.in(RadiansPerSecondPerSecond);

        // apply linear limits
        Translation2d vDes = new Translation2d(des.vxMetersPerSecond, des.vyMetersPerSecond);
        Translation2d vCurr = new Translation2d(curr.vxMetersPerSecond, curr.vyMetersPerSecond);

        // calculate required acceleration
        Translation2d aDes = vDes.minus(vCurr).div(dt);
        aDes = MathTools.clampMagnitude(aDes, aMaxMps2);
        vDes = vCurr.plus(aDes.times(dt));
        vDes = MathTools.clampMagnitude(vDes, vMaxMps);

        // apply angular limits
        double wDes = des.omegaRadiansPerSecond;
        double alphaDes = (wDes - curr.omegaRadiansPerSecond) / dt;
        alphaDes = MathUtil.clamp(alphaDes, -alphaMaxRadps2, alphaMaxRadps2);
        wDes = curr.omegaRadiansPerSecond + alphaDes * dt;
        wDes = MathTools.clampMagnitude(wDes, wMaxRadps);

        // return result
        return new ChassisSpeeds(vDes.getX(), vDes.getY(), wDes);
    }
}