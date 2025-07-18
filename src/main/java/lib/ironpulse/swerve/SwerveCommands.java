package lib.ironpulse.swerve;

import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lib.ironpulse.swerve.commands.SwerveDriveToPose;
import lib.ironpulse.swerve.commands.SwerveFollowPathPlannerTrajectory;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static lib.ironpulse.math.MathTools.epsilonEquals;

public class SwerveCommands {
    private static final Function<Double, Double> kJoystickCurveLinear = (x) -> x;
    private static final Function<Double, Double> kJoystickCurveQuadratic = (x) -> x * x * Math.signum(x);
    private static final Function<Double, Double> kJoystickCurveCubic = (x) -> x * x * x;
    private static final Function<Double, Double> kJoystickCurveSemiCubic = (x) -> Math.pow(x, 2.5);


    public static Command driveWithJoystick(
            Swerve swerve,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier zSupplier,
            Supplier<Pose3d> poseDriveRobotSupplier,
            LinearVelocity translationDeadband,
            AngularVelocity rotationDeadband,
            Function<Double, Double> translationJoystickCurve,
            Function<Double, Double> rotationJoystickCurve
    ) {
        var cmd = Commands.run(() -> {
            SwerveLimit swerveLimit = swerve.getSwerveLimit();

            // read from joystick
            double x = translationJoystickCurve.apply(xSupplier.getAsDouble());
            double y = translationJoystickCurve.apply(ySupplier.getAsDouble());
            double z = rotationJoystickCurve.apply(zSupplier.getAsDouble());

            // compute linear velocity
            double vNorm = MathUtil.applyDeadband(
                    Math.hypot(x, y) * swerveLimit.maxLinearVelocity().in(MetersPerSecond),
                    translationDeadband.in(MetersPerSecond)
            );
            Rotation2d vDir = epsilonEquals(vNorm, 0.0) ? Rotation2d.kZero : new Rotation2d(x, y);
            Translation2d v = new Translation2d(vNorm, vDir);

            // compute angular velocity
            double omegaNorm = MathUtil.applyDeadband(
                    Math.abs(z) * swerveLimit.maxAngularVelocity().in(RadiansPerSecond),
                    rotationDeadband.in(RadiansPerSecond)
            );
            double omegaDir = Math.signum(z);
            AngularVelocity omega = RadiansPerSecond.of(omegaNorm * omegaDir);

            // compose to chassis speeds
            // NOTE: the so-called "FieldRelative" is actually "DriverStationRelative". we take the relative speed
            // of the drivetrain w.r.t DriverStation, based on alliance color
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    v.getX(), v.getY(), omega.in(RadiansPerSecond),
                    poseDriveRobotSupplier.get().getRotation().toRotation2d()
            );

            swerve.runTwist(chassisSpeeds);
        });
        cmd.addRequirements(swerve);

        return cmd;
    }

    public static Command driveWithJoystick(
            Swerve swerve,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier zSupplier,
            Supplier<Pose3d> poseDriverRobotSupplier,
            LinearVelocity translationDeadband,
            AngularVelocity rotationDeadband
    ) {
        return driveWithJoystick(
                swerve, xSupplier, ySupplier, zSupplier, poseDriverRobotSupplier, translationDeadband,
                rotationDeadband,
                kJoystickCurveLinear,
                kJoystickCurveQuadratic
        );
    }

    public static Command driveToPose(
            Swerve swerve,
            Supplier<Pose3d> poseWorldRobotSupplier,
            Supplier<Pose3d> poseWorldTargetSupplier,
            Supplier<Pose2d> velocityWorldRobotSupplier,
            PIDController translationController,
            PIDController rotationController,
            Distance translationTolerance,
            Angle rotationTolerance
    ) {
        return new SwerveDriveToPose(
                swerve, poseWorldRobotSupplier, poseWorldTargetSupplier, velocityWorldRobotSupplier,
                translationController, rotationController, translationTolerance, rotationTolerance
        );
    }

    public static Command stop(Swerve swerve) {
        return Commands.runOnce(swerve::runStop);
    }


    public static Command xLock(Swerve swerve) {
        return Commands.runOnce(swerve::runStopAndLock);
    }

    public static Command reset(Swerve swerve, Pose3d pose) {
        return Commands.runOnce(() -> swerve.resetEstimatedPose(pose));
    }

    public static Command resetAngle(
            Swerve swerve,
            Rotation2d rotation
    ) {
        return Commands.runOnce(() -> {
            Pose3d poseWorldRobotCurr = swerve.getEstimatedPose();
            Pose3d newPoseWorldRobotCurr = new Pose3d(poseWorldRobotCurr.getTranslation(), new Rotation3d(rotation));
            swerve.resetEstimatedPose(newPoseWorldRobotCurr);
        }).ignoringDisable(true);
    }

    public static Command followPathPlannerTrajectory(
            Swerve swerve,
            PathPlannerTrajectory trajectory,
            Supplier<Pose3d> poseWorldRobotSupplier,
            PIDController translationController,
            PIDController rotationController,
            Distance translationTolerance,
            Angle rotationTolerance,
            Consumer<Event> eventConsumer
    ) {
        return new SwerveFollowPathPlannerTrajectory(
                swerve, poseWorldRobotSupplier,
                trajectory,
                translationController, rotationController,
                translationTolerance, rotationTolerance,
                eventConsumer
        );
    }

    public static SysIdRoutine sysid(
            Swerve swerve,
            Velocity<VoltageUnit> rampVelocity,
            Voltage stepVoltage,
            Time timeout
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        rampVelocity, stepVoltage, timeout,
                        state -> {
                            Logger.recordOutput("Sysid/Swerve/Voltage", swerve.getPreviouslyAppliedVoltage());
                            Logger.recordOutput(
                                    "Sysid/Swerve/Position",
                                    swerve.getEstimatedPose().toPose2d().getTranslation().getNorm()
                            );
                            var speed = swerve.getChassisSpeeds();
                            Logger.recordOutput(
                                    "Sysid/Swerve/Velocity",
                                    Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond)
                                            * Math.signum(swerve.getPreviouslyAppliedVoltage().in(Volts))
                            );
                            Logger.recordOutput("Sysid/Swerve/State", state.toString());
                        }
                ),
                new SysIdRoutine.Mechanism(
                        swerve::runVoltage,
                        null,
                        swerve
                )
        );
    }


}
