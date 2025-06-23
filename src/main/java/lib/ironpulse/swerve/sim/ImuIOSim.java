package lib.ironpulse.swerve.sim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import lib.ironpulse.swerve.ImuIO;

public class ImuIOSim implements ImuIO {
    public ImuIOSim() {

    }

    @Override
    public void updateInputs(ImuIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = new Rotation2d();
        inputs.yawVelocityRadPerSec = 0.05;
        inputs.pitchPosition = new Rotation2d();
        inputs.pitchVelocityRadPerSec = 0.05;
        inputs.rollPosition = new Rotation2d();
        inputs.rollVelocityRadPerSec = 0.04;
        inputs.odometryYawTimestamps = new double[]{Timer.getTimestamp()};
        inputs.odometryYawPositions = new Rotation2d[]{inputs.yawPosition};
        inputs.odometryRotations = new Rotation3d[]{
                new Rotation3d(
                        inputs.rollPosition.getMeasure(),
                        inputs.pitchPosition.getMeasure(),
                        inputs.yawPosition.getMeasure()
                )
        };
    }
}
