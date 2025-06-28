package lib.ironpulse.swerve.sjtu6;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import lib.ironpulse.swerve.SwerveConfig;
import lombok.experimental.SuperBuilder;

@SuperBuilder
public class SwerveSJTU6Config extends SwerveConfig {
    public Current driveStatorCurrentLimit;
    public Current steerStatorCurrentLimit;
    public Frequency odometryFrequency;
    public String canivoreCanBusName;
    public int pigeonId;
}
