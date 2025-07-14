package frc.robot.subsystems.photonvision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotStateRecorder;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;
import java.util.Optional;

import static frc.robot.Constants.Photonvision.CAMERA_RELATIVE_TO_ROBOT;
import static frc.robot.Constants.Photonvision.PV_CAMERA_NAMES;

public class PhotonVisionIOReal implements PhotonVisionIO {

    private final String name;
    private final PhotonCamera camera;
    private final int id;
    private Pose3d pose1;
    private Pose3d pose2;

    public PhotonVisionIOReal(int id) {
        this.id = id;
        this.name = PV_CAMERA_NAMES[id];
        camera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(PhotonVisionIOInputs inputs) {
        // Get the latest result for consistent hasTargets status
        List<PhotonPipelineResult> allUnreadResult = camera.getAllUnreadResults();
        boolean hasFreshData = !allUnreadResult.isEmpty();

        // Basic connection info
        inputs.connected = camera.isConnected();
        inputs.name = camera.getName();
        inputs.id = id;
        inputs.hasFreshData = hasFreshData;

        // Process the latest result
        if (hasFreshData) {
            PhotonPipelineResult latestResult = allUnreadResult.get(allUnreadResult.size() - 1);
            Optional<MultiTargetPNPResult> multiTagResult = latestResult.getMultiTagResult();
            inputs.hasTargets = multiTagResult.isPresent();
            inputs.timestampMs = (long) (latestResult.getTimestampSeconds() * 1000);
            if (inputs.hasTargets) {
                Rotation2d oriDegrees = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d().getRotation();
                pose1 = new Pose3d(multiTagResult.get().estimatedPose.best.getTranslation(), multiTagResult.get().estimatedPose.best.getRotation()).transformBy(CAMERA_RELATIVE_TO_ROBOT[id].inverse());
                pose2 = new Pose3d(multiTagResult.get().estimatedPose.alt.getTranslation(), multiTagResult.get().estimatedPose.alt.getRotation()).transformBy(CAMERA_RELATIVE_TO_ROBOT[id].inverse());
                if (pose1.toPose2d().getRotation().minus(oriDegrees).getDegrees() < pose2.toPose2d().getRotation().minus(oriDegrees).getDegrees()) {
                    Logger.recordOutput("PhotonVision Target" + id, new Pose3d(multiTagResult.get().estimatedPose.best.getTranslation(), multiTagResult.get().estimatedPose.best.getRotation()));
                    inputs.bestPose = pose1;
                    inputs.bestPoseReprojErr = multiTagResult.get().estimatedPose.bestReprojErr;
                    inputs.altPose = pose2;
                    inputs.altPoseReprojErr = multiTagResult.get().estimatedPose.altReprojErr;
                } else {
                    Logger.recordOutput("PhotonVision Target" + id, new Pose3d(multiTagResult.get().estimatedPose.alt.getTranslation(), multiTagResult.get().estimatedPose.alt.getRotation()));
                    inputs.bestPose = pose2;
                    inputs.bestPoseReprojErr = multiTagResult.get().estimatedPose.altReprojErr;
                    inputs.altPose = pose1;
                    inputs.altPoseReprojErr = multiTagResult.get().estimatedPose.bestReprojErr;
                }
                inputs.ambiguity = multiTagResult.get().estimatedPose.ambiguity;
            } else {
                inputs.bestPose = new Pose3d();
                inputs.altPose = new Pose3d();
                inputs.bestPoseReprojErr = Double.MAX_VALUE;
                inputs.altPoseReprojErr = Double.MAX_VALUE;
                inputs.ambiguity = 0;
            }
        } else {
            inputs.hasTargets = false;
            inputs.timestampMs = 0;
        }
    }

    @Override
    public void takeOutputSnapshot() {
        camera.takeOutputSnapshot();
    }
}
