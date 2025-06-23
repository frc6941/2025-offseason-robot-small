package lib.ironpulse.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static lib.ironpulse.math.MathTools.epsilonEquals;

public class SwerveCommands {
    private static final double kDeadband = 0.05;
    private static final Function<Double, Double> kJoystickCurveQuadratic = (x) -> x * x;
    private static final Function<Double, Double> kJoystickCurveCubic = (x) -> x * x * x;
    private static final Function<Double, Double> kJoystickCurveSemiCubic = (x) -> Math.pow(x, 2.5);


    public static Command driveWithJoystick(
            Swerve swerve,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier zSupplier,
            Supplier<Pose3d> transformDriverRobotSupplier,
            LinearVelocity translationDeadband,
            AngularVelocity rotationDeadband,
            Function<Translation2d, Translation2d> translationCurve,
            Function<AngularVelocity, AngularVelocity> rotationCurve
    ) {
        var cmd = Commands.run(() -> {
            SwerveLimit swerveLimit = swerve.getSwerveLimit();

            // read from joystick
            double x = xSupplier.getAsDouble();
            double y = ySupplier.getAsDouble();
            double z = zSupplier.getAsDouble();

            // compute linear velocity
            double vNorm = MathUtil.applyDeadband(
                    Math.hypot(x, y) * swerveLimit.maxLinearVelocity().in(MetersPerSecond),
                    translationDeadband.in(MetersPerSecond)
            );
            Rotation2d vDir = epsilonEquals(vNorm, 0.0) ? Rotation2d.kZero : new Rotation2d(x, y);
            Translation2d v = new Translation2d(vNorm, vDir);
            v = translationCurve.apply(v);

            // compute angular velocity
            double omegaNorm = MathUtil.applyDeadband(
                    Math.abs(z) * swerveLimit.maxAngularVelocity().in(RadiansPerSecond),
                    rotationDeadband.in(RadiansPerSecond)
            );
            double omegaDir = Math.signum(z);
            AngularVelocity omega = RadiansPerSecond.of(omegaNorm * omegaDir);
            omega = rotationCurve.apply(omega);

            // compose to chassis speeds
            // NOTE: the so-called "FieldRelative" is actually "DriverStationRelative". we take the relative speed
            // of the drivetrain w.r.t DriverStation, based on alliance color
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    v.getX(), v.getY(), omega.in(RadiansPerSecond),
                    transformDriverRobotSupplier.get().getRotation().toRotation2d()
            );


            swerve.runVelocity(chassisSpeeds);
        });
        cmd.addRequirements(swerve);

        return cmd;
    }

    public static Command driveWithJoystick(
            Swerve swerve,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier zSupplier,
            Supplier<Pose3d> transformDriverRobotSupplier,
            LinearVelocity translationDeadband,
            AngularVelocity rotationDeadband
    ) {
        return driveWithJoystick(
                swerve, xSupplier, ySupplier, zSupplier, transformDriverRobotSupplier, translationDeadband, rotationDeadband,
                (x) -> x, (x) -> x
        );
    }


}
