package lib.ironpulse.swerve.sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import lib.ironpulse.swerve.SwerveConfig;
import lombok.experimental.SuperBuilder;

@SuperBuilder
public class SwerveSimConfig extends SwerveConfig {
    // sim parameters
    DCMotor driveMotor;
    DCMotor steerMotor;
    MomentOfInertia driveMomentOfInertia;
    MomentOfInertia steerMomentOfInertia;
    double driveStdDevPos;
    double driveStdDevVel;
    double steerStdDevPos;
    double steerStdDevVel;
}
