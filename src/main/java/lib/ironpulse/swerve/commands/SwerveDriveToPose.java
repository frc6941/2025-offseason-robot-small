package lib.ironpulse.swerve.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import lib.ironpulse.swerve.Swerve;
import lib.ntext.NTParameter;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static lib.ironpulse.math.MathTools.epsilonEquals;
import static lib.ironpulse.math.MathTools.toAngle;

public class SwerveDriveToPose extends Command {
    private final static String kTag = "Commands/DriveToPose";
    private final Swerve swerve;
    protected Supplier<Pose3d> poseWorldRobotSupplier;
    protected Supplier<Pose3d> poseWorldTargetSupplier;
    protected Supplier<Pose2d> velocityWorldRobotSupplier;
    protected Distance translationTolerance;
    protected Angle rotationTolerance;
    protected PIDController translationController;
    protected PIDController rotationController;

    public SwerveDriveToPose(Swerve swerve,
                             Supplier<Pose3d> poseWorldRobotSupplier,
                             Supplier<Pose3d> poseWorldTargetSupplier,
                             Supplier<Pose2d> velocityWorldRobotSupplier,
                             PIDController translationController,
                             PIDController rotationController,
                             Distance translationTolerance,
                             Angle rotationTolerance) {
        this.swerve = swerve;
        this.poseWorldRobotSupplier = poseWorldRobotSupplier;
        this.poseWorldTargetSupplier = poseWorldTargetSupplier;
        this.velocityWorldRobotSupplier = velocityWorldRobotSupplier;
        this.translationController = translationController;
        this.rotationController = rotationController;
        this.translationTolerance = translationTolerance;
        this.rotationTolerance = rotationTolerance;
        addRequirements(swerve);

        rotationController.enableContinuousInput(0, Math.PI * 2);
    }

    @Override
    public void initialize() {
        translationController.setP(SwerveDriveToPoseParamsNT.translationKp.getValue());
        translationController.setI(SwerveDriveToPoseParamsNT.translationKi.getValue());
        translationController.setD(SwerveDriveToPoseParamsNT.translationKd.getValue());

        rotationController.setP(SwerveDriveToPoseParamsNT.rotationKp.getValue());
        rotationController.setI(SwerveDriveToPoseParamsNT.rotationKi.getValue());
        rotationController.setD(SwerveDriveToPoseParamsNT.rotationKd.getValue());

        translationController.reset();
        rotationController.reset();
    }

    @Override
    public void execute() {
        // get from supplier
        Pose3d TWR = poseWorldRobotSupplier.get();
        Pose3d TWT = poseWorldTargetSupplier.get();
        Pose3d TRT = TWT.relativeTo(TWR);

        // compute translation error, turn into velocity vector
        Translation2d pRT = TRT.toPose2d().getTranslation();
        double pRT_norm = pRT.getNorm();
        Rotation2d pRT_dir = toAngle(pRT);
        // NOTE: as pRT_norm is always positive, then vRT_norm is always negative.
        // to make the robot move along but not opposite to pRT_dir, we take the minus sign before vRT_norm
        double vRT_norm = translationController.calculate(pRT_norm, 0.0);
        Translation2d vRT = new Translation2d(-vRT_norm, pRT_dir);

        // compute rotation err, turn into angular velocity scalar
        double thetaRT = TRT.getRotation().toRotation2d().getRadians();
        double omegaRT = -rotationController.calculate(thetaRT, 0.0);

        // compose and run velocity
        ChassisSpeeds VRT = new ChassisSpeeds(vRT.getX(), vRT.getY(), omegaRT);
        swerve.runTwist(VRT);

        // logging
        Logger.recordOutput(kTag + "/poseWorldTarget", TWR);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.runStop();
    }

    @Override
    public boolean isFinished() {
        Pose3d TWR = poseWorldRobotSupplier.get();
        Pose3d TWT = poseWorldTargetSupplier.get();
        Pose3d TRT = TWT.relativeTo(TWR);

        double tolTransM = translationTolerance.in(Meters);
        double tolRotRad = rotationTolerance.in(Radians);

        return epsilonEquals(TRT.getTranslation().toTranslation2d(), new Translation2d(), tolTransM)
                && epsilonEquals(TRT.getRotation().getAngle(), 0.0, tolRotRad);
    }

    @NTParameter(tableName = "Params/" + kTag)
    public static class SwerveDriveToPoseParams {
        static final double translationKp = 4.5;
        static final double translationKi = 0.0;
        static final double translationKd = 0.0;

        static final double rotationKp = 5.0;
        static final double rotationKi = 0.0;
        static final double rotationKd = 0.0;
        static final double rotationVelocityMax = 5.0;
        static final double rotationAccelerationMax = 20.0;
    }
}

