package lib.ironpulse.swerve;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import lombok.Builder;

/**
 * Enforces drive and steer limit at the module level.
 */
@Builder
public record SwerveModuleLimit(
        LinearVelocity maxDriveVelocity, LinearAcceleration maxDriveAcceleration,
        AngularVelocity maxSteerAngularVelocity, AngularAcceleration maxSteerAngularAcceleration) {
}
