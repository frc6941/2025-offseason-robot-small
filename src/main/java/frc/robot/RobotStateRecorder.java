package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
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

    public static Pose3d getPoseWorldRobotCurrent() {
        return RobotStateRecorder.getInstance().getTransform(
                Seconds.of(Timer.getTimestamp()),
                TransformRecorder.kFrameWorld,
                TransformRecorder.kFrameRobot
        ).orElse(new Pose3d());
    }

    @Override
    public void periodic() {

    }


}
