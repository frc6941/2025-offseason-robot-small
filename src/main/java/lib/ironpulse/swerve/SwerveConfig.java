package lib.ironpulse.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import lombok.Builder;
import lombok.experimental.SuperBuilder;

@SuperBuilder
public class SwerveConfig {
    // general
    public final String name;
    public final double dtS;

    // transmissions
    public final Distance wheelDiameter;
    public final double driveGearRatio;
    public final double steerGearRatio;

    // limits
    public final SwerveLimit defaultSwerveLimit;
    public final SwerveModuleLimit defaultSwerveModuleLimit;

    // modules
    public final SwerveModuleConfig[] moduleConfigs;

    public int moduleCount() {
        return moduleConfigs.length;
    }

    public Translation2d[] moduleLocations() {
        Translation2d[] locations = new Translation2d[moduleCount()];
        for (int i = 0; i < moduleCount(); i++)
            locations[i] = moduleConfigs[i].location;
        return locations;
    }

    @Builder
    public static class SwerveModuleConfig {
        public String name;
        public Translation2d location;
        public int driveMotorId;
        public int steerMotorId;
        public int encoderId;
        public Angle driveMotorEncoderOffset;
        public Angle steerMotorEncoderOffset;
        public boolean driveInverted;
        public boolean steerInverted;
        public boolean encoderInverted;
    }
}
