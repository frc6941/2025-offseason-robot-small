package frc.robot.subsystems.photonvision;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

import static frc.robot.Constants.Photonvision.PV_CAMERA_NAMES;

public class PhotonVisionIOReal implements PhotonVisionIO {

    private final String name;
    private final PhotonCamera camera;
    private final int id;

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
            inputs.hasTargets = latestResult.hasTargets();
            inputs.latencyMs = 0; // Latency method not available in this PhotonVision version
            inputs.timestampMs = (long) (latestResult.getTimestampSeconds() * 1000);

            if (latestResult.hasTargets()) {
                List<PhotonTrackedTarget> targets = latestResult.getTargets();
                inputs.targetCount = targets.size();

                // Initialize arrays for raw detection data
                inputs.targetYaw = new double[targets.size()];
                inputs.targetPitch = new double[targets.size()];
                inputs.targetArea = new double[targets.size()];
                inputs.targetSkew = new double[targets.size()];
                inputs.targetPoseAmbiguity = new double[targets.size()];
                inputs.targetFiducialId = new int[targets.size()];
                inputs.targetPixelX = new double[targets.size()];
                inputs.targetPixelY = new double[targets.size()];

                // Extract raw data from each target
                for (int i = 0; i < targets.size(); i++) {
                    PhotonTrackedTarget target = targets.get(i);

                    inputs.targetYaw[i] = target.getYaw();
                    inputs.targetPitch[i] = target.getPitch();
                    inputs.targetArea[i] = target.getArea();
                    inputs.targetSkew[i] = target.getSkew();
                    inputs.targetPoseAmbiguity[i] = target.getPoseAmbiguity();
                    inputs.targetFiducialId[i] = target.getFiducialId();

                    // Extract pixel coordinates from target center
                    // Note: PhotonVision API may vary by version - adjust method names as needed
                    try {
                        // Try to get pixel coordinates from target center or bounding box
                        // These methods may need adjustment based on actual PhotonVision API
                        var corners = target.getDetectedCorners();
                        if (corners.size() >= 4) {
                            // Calculate center from corners
                            double centerX = corners.stream().mapToDouble(corner -> corner.x).average().orElse(0.0);
                            double centerY = corners.stream().mapToDouble(corner -> corner.y).average().orElse(0.0);
                            inputs.targetPixelX[i] = centerX;
                            inputs.targetPixelY[i] = centerY;
                        } else {
                            // Fallback: use image center as approximate pixel location
                            inputs.targetPixelX[i] = 0.0; // Will be updated when we know image dimensions
                            inputs.targetPixelY[i] = 0.0; // Will be updated when we know image dimensions
                        }
                    } catch (Exception e) {
                        // If pixel coordinate methods are not available, use 0 as placeholder
                        inputs.targetPixelX[i] = 0.0;
                        inputs.targetPixelY[i] = 0.0;
                    }
                }

                // Log raw detection data for debugging - grouped by target
                Logger.recordOutput("PhotonVision/Camera" + id + "/TotalTargets", targets.size());
                Logger.recordOutput("PhotonVision/Camera" + id + "/ResultTimestamp", latestResult.getTimestampSeconds());

                // Log each target's data in its own folder
                for (int i = 0; i < targets.size(); i++) {
                    String targetPath = "PhotonVision/Camera" + id + "/Target" + i;
                    Logger.recordOutput(targetPath + "/Yaw", inputs.targetYaw[i]);
                    Logger.recordOutput(targetPath + "/Pitch", inputs.targetPitch[i]);
                    Logger.recordOutput(targetPath + "/Area", inputs.targetArea[i]);
                    Logger.recordOutput(targetPath + "/Skew", inputs.targetSkew[i]);
                    Logger.recordOutput(targetPath + "/Ambiguity", inputs.targetPoseAmbiguity[i]);
                    Logger.recordOutput(targetPath + "/FiducialId", inputs.targetFiducialId[i]);
                    Logger.recordOutput(targetPath + "/PixelX", inputs.targetPixelX[i]);
                    Logger.recordOutput(targetPath + "/PixelY", inputs.targetPixelY[i]);
                    Logger.recordOutput(targetPath + "/Confidence", Math.max(0.0, 1.0 - inputs.targetPoseAmbiguity[i]));
                }

                // Also log arrays for compatibility
                Logger.recordOutput("PhotonVision/Camera" + id + "/Arrays/TargetYaw", inputs.targetYaw);
                Logger.recordOutput("PhotonVision/Camera" + id + "/Arrays/TargetPitch", inputs.targetPitch);
                Logger.recordOutput("PhotonVision/Camera" + id + "/Arrays/TargetArea", inputs.targetArea);
                Logger.recordOutput("PhotonVision/Camera" + id + "/Arrays/TargetSkew", inputs.targetSkew);
                Logger.recordOutput("PhotonVision/Camera" + id + "/Arrays/TargetAmbiguity", inputs.targetPoseAmbiguity);
                Logger.recordOutput("PhotonVision/Camera" + id + "/Arrays/TargetFiducialId", inputs.targetFiducialId);
                Logger.recordOutput("PhotonVision/Camera" + id + "/Arrays/TargetPixelX", inputs.targetPixelX);
                Logger.recordOutput("PhotonVision/Camera" + id + "/Arrays/TargetPixelY", inputs.targetPixelY);


            } else {
                // No targets detected
                inputs.targetCount = 0;
                inputs.targetYaw = new double[0];
                inputs.targetPitch = new double[0];
                inputs.targetArea = new double[0];
                inputs.targetSkew = new double[0];
                inputs.targetPoseAmbiguity = new double[0];
                inputs.targetFiducialId = new int[0];
                inputs.targetPixelX = new double[0];
                inputs.targetPixelY = new double[0];

                // Log status
                Logger.recordOutput("PhotonVision/Camera" + id + "/TotalTargets", 0);
                Logger.recordOutput("PhotonVision/Camera" + id + "/ResultTimestamp", latestResult.getTimestampSeconds());
            }
        } else {
            // Camera not returning results (likely disconnected or serious error)
            inputs.hasTargets = false;
            inputs.targetCount = 0;
            inputs.latencyMs = 0;
            inputs.timestampMs = System.currentTimeMillis();

            // Empty arrays for no results
            inputs.targetYaw = new double[0];
            inputs.targetPitch = new double[0];
            inputs.targetArea = new double[0];
            inputs.targetSkew = new double[0];
            inputs.targetPoseAmbiguity = new double[0];
            inputs.targetFiducialId = new int[0];
            inputs.targetPixelX = new double[0];
            inputs.targetPixelY = new double[0];

            // Log status
            Logger.recordOutput("PhotonVision/Camera" + id + "/TotalTargets", 0);
            Logger.recordOutput("PhotonVision/Camera" + id + "/NoResults", true);
        }

        // Log freshness status
        Logger.recordOutput("PhotonVision/Camera" + id + "/HasFreshData", hasFreshData);
    }

    @Override
    public void takeOutputSnapshot() {
        camera.takeOutputSnapshot();
    }
}
