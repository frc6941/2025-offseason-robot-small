package frc.robot.subsystems.photonvision;

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
        inputs.targetCount = 0;
        inputs.latencyMs = 0;
        inputs.timestampMs = System.currentTimeMillis();

        // Empty arrays for no targets
        inputs.targetYaw = new double[0];
        inputs.targetPitch = new double[0];
        inputs.targetArea = new double[0];
        inputs.targetSkew = new double[0];
        inputs.targetPoseAmbiguity = new double[0];
        inputs.targetFiducialId = new int[0];
        inputs.targetPixelX = new double[0];
        inputs.targetPixelY = new double[0];

        // TODO: Implement simulation with random target detections based on alliance color
        // This could include generating simulated raw detection data for testing
    }

    @Override
    public void takeOutputSnapshot() {
        // No-op for simulation
    }
}
