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

    @AutoLog
    class PhotonVisionIOInputs {
        public String name;
        public int id;
        public boolean connected = false;
        public boolean hasTargets = false;
        public boolean hasFreshData = false;  // Indicates if this update contains new vision data
        public long timestampMs = 0;

        public Pose3d bestPose = new Pose3d();
        public double bestPoseReprojErr = 0;
        public double ambiguity = 0;
        public Pose3d altPose = new Pose3d();
        public double altPoseReprojErr = 0;
    }

}