package frc.robot.subsystems.photonvision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.FieldConstants;
import org.littletonrobotics.AllianceFlipUtil;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;
import java.util.Optional;

import static frc.robot.Constants.Photonvision.CAMERA_RELATIVE_TO_ROBOT;
import static frc.robot.Constants.Photonvision.PV_CAMERA_NAMES;

public class PhotonVisionIOReal implements PhotonVisionIO {

    private final String name;
    private final PhotonCamera camera;
    private final int id;
    private PhotonPoseEstimator poseEstimator;


    public PhotonVisionIOReal(int id) {
        this.id = id;
        this.name = PV_CAMERA_NAMES[id];
        camera = new PhotonCamera(name);
        poseEstimator = new PhotonPoseEstimator(
                FieldConstants.officialAprilTagType.getLayout(),
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                CAMERA_RELATIVE_TO_ROBOT[id]);
        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        poseEstimator.setFieldTags(FieldConstants.weldedReefAprilTagType.getLayout());
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
            Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update(latestResult);
            if (estimatedRobotPose.isPresent()) {
                inputs.hasTargets = true;
                inputs.estimatePose = estimatedRobotPose.get().estimatedPose;
                inputs.strategyUsed = switch (estimatedRobotPose.get().strategy) {
                    case MULTI_TAG_PNP_ON_COPROCESSOR -> estimateStrategy.MULTI_TAG;
                    case CLOSEST_TO_REFERENCE_POSE -> estimateStrategy.SINGLE_TAG;
                    default -> estimateStrategy.None;
                };
                inputs.timestampSec = estimatedRobotPose.get().timestampSeconds;
            }
        } else {
            inputs.hasTargets = false;
            inputs.timestampSec = 0;
            inputs.estimatePose = new Pose3d();
            inputs.strategyUsed = estimateStrategy.None;
        }
    }

    @Override
    public void takeOutputSnapshot() {
        camera.takeOutputSnapshot();
    }

    @Override
    public void setRefPose(Pose3d refPose) {
        poseEstimator.setReferencePose(refPose);
    }

    @Override
    public void updateLayout() {
        poseEstimator.setFieldTags(AllianceFlipUtil.shouldFlip() ?
                FieldConstants.weldedRedAprilTagType.getLayout() :
                FieldConstants.weldedBlueAprilTagType.getLayout());
    }

}
