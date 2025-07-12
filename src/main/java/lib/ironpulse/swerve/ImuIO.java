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
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityRadPerSec = 0.0;
        public double yawVelocityRadPerSecCmd = 0.0;
        public Rotation2d pitchPosition = new Rotation2d();
        public double pitchVelocityRadPerSec = 0.0;
        public Rotation2d rollPosition = new Rotation2d();
        public double rollVelocityRadPerSec = 0.0;
        public double[] odometryYawTimestamps = new double[0];
        public Rotation2d[] odometryYawPositions = new Rotation2d[0];
        public Rotation3d[] odometryRotations = new Rotation3d[0];
    }

}
