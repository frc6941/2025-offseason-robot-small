package frc.robot.subsystems.photonvision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/**
 * PhotonVision IO interface for raw object detection data.
 * DESIGNED FOR LOCALIZATION ONLY, DOES NOT CONTAIN OBJECT DETECTION
 */
public interface PhotonVisionIO {

    default void updateInputs(PhotonVisionIOInputs inputs) {
    }

    void takeOutputSnapshot();

    void setRefPose(Pose3d refPose);

    void updateLayout();

    @AutoLog
    class PhotonVisionIOInputs {
        public String name;
        public int id;
        public boolean connected = false;
        public boolean hasTargets = false;
        public boolean hasFreshData = false;  // Indicates if this update contains new vision data
        public double timestampSec = 0;

        public Pose3d estimatePose = new Pose3d();
        public estimateStrategy strategyUsed = estimateStrategy.None;
    }

    public enum estimateStrategy {
        None, MULTI_TAG, SINGLE_TAG,
    }
}