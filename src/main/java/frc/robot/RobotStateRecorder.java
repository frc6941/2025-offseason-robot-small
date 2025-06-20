package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import lib.ironpulse.rbd.TransformRecorder;

import static edu.wpi.first.units.Units.Seconds;

public class RobotStateRecorder extends TransformRecorder {
    private static RobotStateRecorder instance;
    private SwerveDrivePoseEstimator3d swervePoseEstimator;

    private RobotStateRecorder() {
        putTransform(kTransformWorldDriverStationBlue, kFrameWorld, kFrameDriverStationBlue); // static: TWorldDSB
        putTransform(kTransformWorldDriverStationRed, kFrameWorld, kFrameDriverStationRed); // static TWorldDSR
        putTransform(new Pose3d(), Seconds.of(0.0), kFrameWorld, kFrameRobot); // dynamic TWorldRobot at origin
    }

    public static RobotStateRecorder getInstance() {
        if (instance == null) {
            instance = new RobotStateRecorder();
        }
        return instance;
    }

}
