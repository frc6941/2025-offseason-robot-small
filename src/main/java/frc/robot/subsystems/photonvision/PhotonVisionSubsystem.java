package frc.robot.subsystems.photonvision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PhotonVisionParamsNT;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.Constants.Photonvision.SNAPSHOT_ENABLED;
import static frc.robot.Constants.Photonvision.SNAPSHOT_PERIOD;

public class PhotonVisionSubsystem extends SubsystemBase {

    private final PhotonVisionIO[] ios;
    private final PhotonVisionIOInputsAutoLogged[] inputs;
    private Timer snapshotTimer = new Timer();

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
    }


    /**
     * Gets all raw detection data from all cameras
     *
     * @return List of all raw detections with camera information
     */
    public List<RawDetection> getAllRawDetections() {
        List<RawDetection> allDetections = new ArrayList<>();

        for (int i = 0; i < inputs.length; i++) {
            PhotonVisionIOInputsAutoLogged input = inputs[i];
            if (input.hasTargets && input.targetCount > 0) {
                for (int j = 0; j < input.targetCount; j++) {
                    double yaw = j < input.targetYaw.length ? input.targetYaw[j] : 0.0;
                    double pitch = j < input.targetPitch.length ? input.targetPitch[j] : 0.0;
                    double area = j < input.targetArea.length ? input.targetArea[j] : 0.0;
                    double skew = j < input.targetSkew.length ? input.targetSkew[j] : 0.0;
                    double ambiguity = j < input.targetPoseAmbiguity.length ? input.targetPoseAmbiguity[j] : 1.0;
                    int fiducialId = j < input.targetFiducialId.length ? input.targetFiducialId[j] : -1;
                    double pixelX = j < input.targetPixelX.length ? input.targetPixelX[j] : 0.0;
                    double pixelY = j < input.targetPixelY.length ? input.targetPixelY[j] : 0.0;

                    allDetections.add(new RawDetection(
                            i, // camera ID
                            input.timestampMs,
                            yaw,
                            pitch,
                            area,
                            skew,
                            ambiguity,
                            fiducialId,
                            pixelX,
                            pixelY
                    ));
                }
            }
        }

        return allDetections;
    }

    /**
     * Checks if any camera has targets
     *
     * @return true if any camera has targets, false otherwise
     */
    public boolean hasAnyTargets() {
        for (PhotonVisionIOInputsAutoLogged input : inputs) {
            if (input.hasTargets) {
                return true;
            }
        }
        return false;
    }

    /**
     * Transforms a pose from camera coordinates to robot coordinates
     *
     * @param cameraRelativePose The pose relative to the camera
     * @return The pose relative to the robot center
     */
    private Pose3d transformCameraToRobot(Pose3d cameraRelativePose) {
        // Get camera position relative to robot center from constants
        double cameraToRobotX = PhotonVisionParamsNT.CAMERA_TO_ROBOT_X.getValue();
        double cameraToRobotY = PhotonVisionParamsNT.CAMERA_TO_ROBOT_Y.getValue();
        double cameraToRobotZ = PhotonVisionParamsNT.CAMERA_TO_ROBOT_Z.getValue();
        double cameraToRobotRotationDegrees = PhotonVisionParamsNT.CAMERA_TO_ROBOT_ROTATION_DEGREES.getValue();

        // Create the camera-to-robot transform
        Translation3d cameraToRobotTranslation = new Translation3d(cameraToRobotX, cameraToRobotY, cameraToRobotZ);
        Rotation3d cameraToRobotRotation = new Rotation3d(0, 0, Units.degreesToRadians(cameraToRobotRotationDegrees));
        Transform3d cameraToRobotTransform = new Transform3d(cameraToRobotTranslation, cameraToRobotRotation);

        // Apply the transform to get robot-relative pose
        return cameraRelativePose.transformBy(cameraToRobotTransform);
    }


    /**
     * Record class to hold raw detection data
     */
    public record RawDetection(
            int cameraId,
            long timestampMs,
            double yaw,         // degrees, positive = right
            double pitch,       // degrees, positive = up
            double area,        // percent (0-100)
            double skew,        // degrees
            double ambiguity,   // 0-1, lower is better
            int fiducialId,     // -1 for objects, positive for AprilTags
            double pixelX,      // pixel X coordinate of target center
            double pixelY       // pixel Y coordinate of target center
    ) {
    }
}
