package frc.robot.subsystems.photonvision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateRecorder;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Photonvision.SNAPSHOT_ENABLED;
import static frc.robot.Constants.Photonvision.SNAPSHOT_PERIOD;

public class PhotonVisionSubsystem extends SubsystemBase {

    private final PhotonVisionIO[] ios;
    private final PhotonVisionIOInputsAutoLogged[] inputs;
    private final Timer snapshotTimer = new Timer();
    @AutoLogOutput(key = "PhotonVision/finalEstimatedPose")
    public Pose3d estimatedPose = null;
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
            ios[i].setRefPose(RobotStateRecorder.getPoseWorldRobotCurrent());
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("PhotonVision/Inst" + i, inputs[i]);
            SmartDashboard.putString("PhotonVision/Inst" + i + "/Name", inputs[i].name);
            SmartDashboard.putString("PhotonVision/Inst" + i + "/StrategyUsed", inputs[i].strategyUsed.toString());
            SmartDashboard.putBoolean("PhotonVision/Inst" + i + "/Connected", inputs[i].connected);
            SmartDashboard.putBoolean("PhotonVision/Inst" + i + "/hasFreshData", inputs[i].hasFreshData);
            SmartDashboard.putBoolean("PhotonVision/Inst" + i + "/hasTargets", inputs[i].hasTargets);
        }
        if (snapshotTimer.hasElapsed(SNAPSHOT_PERIOD)) {
            for (int i = 0; i < ios.length; i++) {
                if (SNAPSHOT_ENABLED[i]) ios[i].takeOutputSnapshot();
            }
            snapshotTimer.reset();
        }
        determinePoseEstimate();
    }

    public void determinePoseEstimate() {

        Pose3d bestPose = null;
        boolean multitagUsed = false;
        int idSelected = -1;
        for (int i = 0; i < ios.length; i++) {
            switch (inputs[i].strategyUsed) {
                case MULTI_TAG: {
                    if (!multitagUsed || replacePose(bestPose, inputs[i].estimatePose)) {
                        multitagUsed = true;
                        bestPose = inputs[i].estimatePose;
                        idSelected = i;
                    }
                    break;
                }
                case SINGLE_TAG:
                    if (!multitagUsed && (bestPose == null || replacePose(bestPose, inputs[i].estimatePose))) {
                        bestPose = inputs[i].estimatePose;
                        idSelected = i;
                    }
                    break;
                default:
            }

        }
        if (idSelected != -1) {
            estimatedPose = inputs[idSelected].estimatePose;
            timestampSeconds = inputs[idSelected].timestampSec;
        } else {
            estimatedPose = null;
            timestampSeconds = 0.0;
        }
        return;
    }

    public boolean replacePose(Pose3d poseCompared, Pose3d poseSelected) {
//        Logger.recordOutput("PV/PoseC", poseCompared);
//        Logger.recordOutput("PV/PoseS", poseSelected);
        return Math.abs(poseCompared.getRotation().minus(
                RobotStateRecorder.getPoseWorldRobotCurrent().getRotation()).getAngle()) <
                Math.abs(poseSelected.getRotation().minus(
                        RobotStateRecorder.getPoseWorldRobotCurrent().getRotation()).getAngle());
    }
}
