package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import lib.ironpulse.rbd.TransformRecorder;

import static edu.wpi.first.units.Units.Seconds;

public class RobotStateRecorder extends TransformRecorder implements Subsystem {
    private static RobotStateRecorder instance;

    private RobotStateRecorder() {
        // add default transforms
        putTransform(kTransformWorldDriverStationBlue, kFrameWorld, kFrameDriverStationBlue); // static: TWorldDSB
        putTransform(kTransformWorldDriverStationRed, kFrameWorld, kFrameDriverStationRed); // static TWorldDSR
        putTransform(new Pose3d(), Seconds.of(0.0), kFrameWorld, kFrameRobot); // dynamic TWorldRobot at origin

        // register update
        CommandScheduler.getInstance().registerSubsystem(new Subsystem[]{this});
    }

    public static RobotStateRecorder getInstance() {
        if (instance == null) {
            instance = new RobotStateRecorder();
        }
        return instance;
    }

    @Override
    public void periodic() {

    }


}
