package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import lib.ironpulse.rbd.TransformRecorder;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Seconds;
import static lib.ironpulse.math.MathTools.toPose2d;

public class RobotStateRecorder extends TransformRecorder {
    private static RobotStateRecorder instance;
    private static TimeInterpolatableBuffer<Pose2d> velocityRobotBuffer;

    private RobotStateRecorder() {
        setBufferDuration(2.0);
        velocityRobotBuffer = TimeInterpolatableBuffer.createBuffer(2.0);

        // add default transforms
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

    public static void periodic() {
        Logger.recordOutput("RobotStateRecorder/fixed",new Pose3d());
        // logging
        Logger.recordOutput("RobotStateRecorder/poseWorldRobot", RobotStateRecorder.getPoseWorldRobotCurrent());
        Logger.recordOutput("RobotStateRecorder/velocityRobot", RobotStateRecorder.getVelocityRobotCurrent());
        Logger.recordOutput("RobotStateRecorder/velocityWorldRobot", RobotStateRecorder.getVelocityWorldRobotCurrent());
    }

    public static void putVelocityRobot(Time time, ChassisSpeeds speed) {
        velocityRobotBuffer.addSample(time.in(Seconds), toPose2d(speed));
    }

    public static Pose2d getVelocityRobotCurrent() {
        return velocityRobotBuffer.getSample(Timer.getTimestamp()).orElse(new Pose2d());
    }

    public static Pose2d getVelocityWorldRobotCurrent() {
        // robot-relative velocity (dx, dy, dθ) and current robot pose in world
        Pose2d velocityRobot = getVelocityRobotCurrent();
        Pose3d poseWorldRobot = getPoseWorldRobotCurrent();

        // drop to 2D to get the robot's heading in the XY plane
        Pose2d pose2dWR = poseWorldRobot.toPose2d();
        Translation2d velRobotTrans = velocityRobot.getTranslation();

        // rotate the translational velocity by the robot’s heading
        Translation2d velWorldTrans = velRobotTrans.rotateBy(pose2dWR.getRotation());

        // preserve the same angular component
        return new Pose2d(velWorldTrans, velocityRobot.getRotation());
    }

    public static Pose3d getPoseWorldRobotCurrent() {
        return RobotStateRecorder.getInstance().getTransform(
                Seconds.of(Timer.getTimestamp()),
                TransformRecorder.kFrameWorld,
                TransformRecorder.kFrameRobot
        ).orElse(new Pose3d());
    }

    public static Pose3d getPoseDriverRobotCurrent() {
        return RobotStateRecorder.getInstance().getTransform(
                Seconds.of(Timer.getTimestamp()),
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(
                        DriverStation.Alliance.Blue) ? RobotStateRecorder.kFrameDriverStationBlue
                        : RobotStateRecorder.kFrameDriverStationRed,
                TransformRecorder.kFrameRobot
        ).orElse(new Pose3d());
    }
}
