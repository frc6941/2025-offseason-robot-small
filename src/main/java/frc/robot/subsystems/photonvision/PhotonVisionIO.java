package frc.robot.subsystems.photonvision;

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
        public int targetCount = 0;
        public long latencyMs = 0;
        public long timestampMs = 0;

        // Raw detection data arrays (one entry per target)
        public double[] targetYaw = new double[0];      // tx - horizontal offset in degrees
        public double[] targetPitch = new double[0];    // ty - vertical offset in degrees
        public double[] targetArea = new double[0];     // ta - area percentage (0-100)
        public double[] targetSkew = new double[0];     // ts - skew in degrees
        public double[] targetPoseAmbiguity = new double[0]; // ambiguity (0-1, lower is better)
        public int[] targetFiducialId = new int[0];     // fiducial ID (for AprilTags, -1 for objects)
        public double[] targetPixelX = new double[0];   // pixel X coordinate of target center
        public double[] targetPixelY = new double[0];   // pixel Y coordinate of target center
    }

}