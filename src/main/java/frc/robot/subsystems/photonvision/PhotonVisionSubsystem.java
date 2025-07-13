package frc.robot.subsystems.photonvision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

import static frc.robot.Constants.Photonvision.SNAPSHOT_ENABLED;
import static frc.robot.Constants.Photonvision.SNAPSHOT_PERIOD;

public class PhotonVisionSubsystem extends SubsystemBase {

    private final PhotonVisionIO[] ios;
    private final PhotonVisionIOInputsAutoLogged[] inputs;
    private Timer snapshotTimer = new Timer();
    public Optional<Pose3d> estimatedPose = Optional.empty();
    public double timestampSeconds = 0.0;

    public PhotonVisionSubsystem(PhotonVisionIO... ios) {
        this.ios = ios;
        inputs = new PhotonVisionIOInputsAutoLogged[ios.length];
        for (int i = 0; i < ios.length; i++) {
            inputs[i] = new PhotonVisionIOInputsAutoLogged();
        }
        snapshotTimer.start();
    }

    @Override
    public void periodic() {
        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("PhotonVision/Inst" + i, inputs[i]);
        }
        if (snapshotTimer.hasElapsed(SNAPSHOT_PERIOD)) {
            for (int i = 0; i < ios.length; i++) {
                if (SNAPSHOT_ENABLED[i]) ios[i].takeOutputSnapshot();
            }
            snapshotTimer.reset();
        }
        estimatedPose = determinePoseEstimate();
        System.out.println(estimatedPose);
    }

    public Optional<Pose3d> determinePoseEstimate() {

        Pose3d bestPose = null;
        double bestErr = Double.MAX_VALUE;
        for (int i = 0; i < ios.length; i++) {
            if (inputs[i].bestPoseReprojErr < bestErr) {
                bestErr = inputs[i].bestPoseReprojErr;
                bestPose = inputs[i].bestPose;
                timestampSeconds = (double) inputs[i].timestampMs / 1000;
            }
        }
        if (bestPose == null) {
            return Optional.empty();
        }
        return Optional.of(bestPose);
    }
}
