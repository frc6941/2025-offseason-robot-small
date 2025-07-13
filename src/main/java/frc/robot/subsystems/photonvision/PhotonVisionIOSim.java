package frc.robot.subsystems.photonvision;

import edu.wpi.first.math.geometry.Pose3d;

import static frc.robot.Constants.Photonvision.PV_CAMERA_NAMES;

public class PhotonVisionIOSim implements PhotonVisionIO {

    private boolean connected = true;
    private String name;
    private int id;

    public PhotonVisionIOSim(int id) {
        this.id = id;
        this.name = PV_CAMERA_NAMES[id];
    }

    @Override
    public void updateInputs(PhotonVisionIOInputs inputs) {
        inputs.connected = connected;
        inputs.name = name;
        inputs.id = id;
        inputs.hasTargets = false;
        inputs.timestampMs = System.currentTimeMillis();

        inputs.hasFreshData = false;
        inputs.bestPose = new Pose3d();
        inputs.altPose = new Pose3d();
        inputs.bestPoseReprojErr = 0;
        inputs.altPoseReprojErr = 0;
        inputs.ambiguity = 0;
    }

    @Override
    public void takeOutputSnapshot() {
        // No-op for simulation
    }
}
