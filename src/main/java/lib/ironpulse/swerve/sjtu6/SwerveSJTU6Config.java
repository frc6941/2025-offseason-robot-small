package lib.ironpulse.swerve.sjtu6;

import edu.wpi.first.units.measure.Current;
import lib.ironpulse.swerve.SwerveConfig;
import lombok.experimental.SuperBuilder;

@SuperBuilder
public class SwerveSJTU6Config extends SwerveConfig {
    // slip configurations
    Current slipCurrent;


}
