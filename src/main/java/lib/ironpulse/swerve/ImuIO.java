package lib.ironpulse.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ImuIO {
    // read
    default void updateInputs(ImuIOInputs inputs) {
    }

    @AutoLog
    class ImuIOInputs {
        boolean connected;
        Rotation2d yawPosition;
        double yawVelocityRadPerSec;
        Rotation2d pitchPosition;
        double pitchVelocityRadPerSec;
        Rotation2d rollPosition;
        double rollVelocityRadPerSeC;
        double[] odometryYawTimestamps;
        Rotation2d[] odometryYawPositions;
    }

}
