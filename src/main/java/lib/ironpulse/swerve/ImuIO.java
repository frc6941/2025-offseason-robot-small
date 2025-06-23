package lib.ironpulse.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

public interface ImuIO {
    // read
    default void updateInputs(ImuIOInputs inputs) {
    }


    @AutoLog
    class ImuIOInputs {
        public boolean connected;
        public Rotation2d yawPosition;
        public double yawVelocityRadPerSec;
        public Rotation2d pitchPosition;
        public double pitchVelocityRadPerSec;
        public Rotation2d rollPosition;
        public double rollVelocityRadPerSec;
        public double[] odometryYawTimestamps;
        public Rotation2d[] odometryYawPositions;
        public Rotation3d[] odometryRotations;
    }

}
