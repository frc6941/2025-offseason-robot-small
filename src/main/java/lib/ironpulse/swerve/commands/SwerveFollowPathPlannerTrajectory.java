package lib.ironpulse.swerve.commands;

import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import lib.ironpulse.swerve.Swerve;
import lombok.Setter;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.Consumer;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static lib.ironpulse.math.MathTools.epsilonEquals;
import static lib.ironpulse.math.MathTools.toAngle;

public class SwerveFollowPathPlannerTrajectory extends Command {
    private final Swerve swerve;
    private final PathPlannerTrajectory trajectory;
    private final Supplier<Pose3d> poseWorldRobotSupplier;
    private final Consumer<Event> eventConsumer;
    private final Timer trajectoryTimer = new Timer();
    @Setter
    private PIDController translationController;
    @Setter
    private PIDController rotationController;
    @Setter
    private Distance translationTolerance;
    @Setter
    private Angle rotationTolerance;
    @Setter
    private EndStrategy endStrategy = EndStrategy.EndWithTime;

    private Queue<Event> eventQueue = new LinkedList<>();

    public SwerveFollowPathPlannerTrajectory(Swerve swerve, Supplier<Pose3d> poseWorldRobotSupplier,
                                             PathPlannerTrajectory trajectory, PIDController translationController,
                                             PIDController rotationController, Distance translationTolerance,
                                             Angle rotationTolerance, Consumer<Event> eventConsumer

    ) {
        // initialize
        this.swerve = swerve;
        this.trajectory = trajectory;
        this.poseWorldRobotSupplier = poseWorldRobotSupplier;
        this.translationController = translationController;
        this.rotationController = rotationController;
        this.translationTolerance = translationTolerance;
        this.rotationTolerance = rotationTolerance;
        this.eventConsumer = eventConsumer;
        addRequirements(swerve);

        // sort through the events by time, so at runtime polling is easy
        var events = trajectory.getEvents();
        events.sort((x, y) -> {
            double t1 = x.getTimestampSeconds();;
            double t2 = y.getTimestampSeconds();
            if (t1 < t2) return -1;
            else if (t1 > t2) return 1;
            return 0;
        });
        eventQueue = new LinkedList<>(events);
    }

    public SwerveFollowPathPlannerTrajectory(
            Swerve swerve, Supplier<Pose3d> poseWorldRobotSupplier,
            PathPlannerTrajectory trajectory, PIDController translationController,
            PIDController rotationController, Distance translationTolerance,
            Angle rotationTolerance
    ) {
        this(swerve, poseWorldRobotSupplier, trajectory, translationController, rotationController, translationTolerance, rotationTolerance, null);
    }


    @Override
    public void initialize() {
        translationController.reset();
        rotationController.reset();
        trajectoryTimer.reset();
        trajectoryTimer.start();
    }

    @Override
    public void execute() {
        // get current time, sample current pose
        double t = trajectoryTimer.get();
        Pose2d TWR = poseWorldRobotSupplier.get().toPose2d();
        PathPlannerTrajectoryState trajectoryState = trajectory.sample(t);

        // get velocity and force feedforward
        ChassisSpeeds V_FF = trajectoryState.fieldSpeeds;
        Current[] tau_FF = trajectoryState.feedforwards.torqueCurrents();

        // get velocity feedback
        // compute translation error, turn into velocity vector
        Pose2d TWT = trajectoryState.pose;
        Pose2d TRT = TWT.relativeTo(TWR);
        Translation2d pRT = TRT.getTranslation();
        double pRT_norm = pRT.getNorm();
        Rotation2d pRT_dir = toAngle(pRT);
        // NOTE: as pRT_norm is always positive, then vRT_norm is always negative.
        // to make the robot move along but not opposite to pRT_dir, we take the minus sign before vRT_norm
        double vRT_norm = translationController.calculate(pRT_norm, 0.0);
        Translation2d vRT = new Translation2d(-vRT_norm, pRT_dir);
        // compute rotation err, turn into angular velocity scalar
        double thetaRT = TRT.getRotation().getRadians();
        double omegaRT = rotationController.calculate(thetaRT, 0.0);
        // compose feedback velocity
        ChassisSpeeds V_FB = new ChassisSpeeds(vRT.getX(), vRT.getY(), omegaRT);

        // compose final command (vel_ff + tau_ff + vel_fb), send to swerve
        swerve.runTwistWithTorque(V_FF.plus(V_FB), tau_FF);

        // handle events
        while (eventConsumer !=null && !eventQueue.isEmpty() && eventQueue.peek().getTimestampSeconds() <= t)
            eventConsumer.accept(eventQueue.poll());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.runTwist(trajectory.getEndState().fieldSpeeds);
    }

    @Override
    public boolean isFinished() {
        boolean isTimeout = trajectoryTimer.hasElapsed(trajectory.getTotalTimeSeconds());

        if (endStrategy.equals(EndStrategy.EndWithTimeAndPose)) {
            Pose2d poseWorldRobotCurrent = poseWorldRobotSupplier.get().toPose2d();
            Pose2d poseWorldTrajectoryEnd = trajectory.getEndState().pose;
            boolean isOnTarget = epsilonEquals(
                    poseWorldRobotCurrent.getTranslation(), poseWorldTrajectoryEnd.getTranslation(),
                    translationTolerance.in(Meters)
            ) && epsilonEquals(
                    poseWorldRobotCurrent.getRotation(), poseWorldTrajectoryEnd.getRotation(),
                    rotationTolerance.in(Radians)
            );
            return isTimeout && isOnTarget;
        }

        return isTimeout;
    }

    public enum EndStrategy {
        EndWithTime, EndWithTimeAndPose
    }
}
