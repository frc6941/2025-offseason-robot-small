package lib.ironpulse.swerve.commands;

import com.pathplanner.lib.events.Event;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import lib.ironpulse.swerve.Swerve;
import lombok.Setter;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static lib.ironpulse.math.MathTools.epsilonEquals;
import static lib.ironpulse.math.MathTools.toAngle;

public class SwerveFollowTrajectory extends Command {
    private final Swerve swerve;
    private final Trajectory trajectory;
    private final Supplier<Pose3d> poseWorldRobotSupplier;
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

    public SwerveFollowTrajectory(Swerve swerve, Supplier<Pose3d> poseWorldRobotSupplier,
                                             Trajectory trajectory, PIDController translationController,
                                             PIDController rotationController, Distance translationTolerance,
                                             Angle rotationTolerance

    ) {
        // initialize
        this.swerve = swerve;
        this.trajectory = trajectory;
        this.poseWorldRobotSupplier = poseWorldRobotSupplier;
        this.translationController = translationController;
        this.rotationController = rotationController;
        this.translationTolerance = translationTolerance;
        this.rotationTolerance = rotationTolerance;
        addRequirements(swerve);
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
        Trajectory.State trajectoryState = trajectory.sample(t);

        // get velocity feedback
        // compute translation error, turn into velocity vector
        Pose2d TWT = trajectoryState.poseMeters;
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

        // compose final command (vel_fb), send to swerve
        swerve.runTwist(V_FB);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.runStop();
    }

    @Override
    public boolean isFinished() {
        boolean isTimeout = trajectoryTimer.hasElapsed(trajectory.getTotalTimeSeconds());

        if (endStrategy.equals(EndStrategy.EndWithTimeAndPose)) {
            Pose2d poseWorldRobotCurrent = poseWorldRobotSupplier.get().toPose2d();
            Pose2d poseWorldTrajectoryEnd = trajectory.getStates().get(trajectory.getStates().size()).poseMeters;
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
