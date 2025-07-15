package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ReefAimCommandParamsNT;
import frc.robot.RobotStateRecorder;
import frc.robot.drivers.AimGoalSupplier;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import lib.ironpulse.math.MathTools;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ironpulse.utils.Logging;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.units.Units.*;
import static frc.robot.drivers.DestinationSupplier.elevatorSetpoint.L4;
import static lib.ironpulse.math.MathTools.epsilonEquals;

public class ReefAimCommand extends Command {
    private final static String kTag = "Commands/ReefAimCommand";
    private final Swerve swerve;
    private final IndicatorSubsystem indicatorSubsystem;
    private boolean rightReef; // true if shooting right reef
    private boolean xOnTarget, xStationary, yOnTarget, yStationary, rotationOnTarget, rotationStationary, imuPitchStable, imuRollStable;
    private Pose2d poseWorldRobot, velocityRobot, tagPose, poseWorldTarget, finalDestinationPose;
    private PIDController translationController;
    private PIDController rotationController;

    private final boolean useSelectedTarget;

    public ReefAimCommand(Swerve swerve, IndicatorSubsystem indicatorSubsystem, boolean useSelectedTarget) {
        this.indicatorSubsystem = indicatorSubsystem;
        this.swerve = swerve;

        translationController = new PIDController(
                ReefAimCommandParamsNT.translationKp.getValue(),
                ReefAimCommandParamsNT.translationKi.getValue(),
                ReefAimCommandParamsNT.translationKd.getValue()
        );
        rotationController = new PIDController(
                ReefAimCommandParamsNT.rotationKp.getValue(),
                ReefAimCommandParamsNT.rotationKi.getValue(),
                ReefAimCommandParamsNT.rotationKd.getValue()
        );
        addRequirements(swerve);

        this.useSelectedTarget = useSelectedTarget;
    }

    public ReefAimCommand(Swerve swerve, IndicatorSubsystem indicatorSubsystem) {
        this(swerve, indicatorSubsystem, false);
    }

    private boolean useFast() {
        return DestinationSupplier.getInstance().getCurrentElevSetpointCoral() != L4;
    }

    @Override
    public void initialize() {
        // tuning
        if (useFast()) {
            translationController.setP(ReefAimCommandParamsNT.translationFastKp.getValue());
            translationController.setI(ReefAimCommandParamsNT.translationFastKi.getValue());
            translationController.setIZone(ReefAimCommandParamsNT.translationFastKiZone.getValue());
            translationController.setD(ReefAimCommandParamsNT.translationFastKd.getValue());

            rotationController.setP(ReefAimCommandParamsNT.rotationKp.getValue());
            rotationController.setI(ReefAimCommandParamsNT.rotationKi.getValue());
            rotationController.setIZone(ReefAimCommandParamsNT.rotationKiZone.getValue());
            rotationController.setD(ReefAimCommandParamsNT.rotationKd.getValue());
        } else {
            translationController.setP(ReefAimCommandParamsNT.translationKp.getValue());
            translationController.setI(ReefAimCommandParamsNT.translationKi.getValue());
            translationController.setIZone(ReefAimCommandParamsNT.translationKiZone.getValue());
            translationController.setD(ReefAimCommandParamsNT.translationKd.getValue());

            rotationController.setP(ReefAimCommandParamsNT.rotationKp.getValue());
            rotationController.setI(ReefAimCommandParamsNT.rotationKi.getValue());
            rotationController.setIZone(ReefAimCommandParamsNT.rotationKiZone.getValue());
            rotationController.setD(ReefAimCommandParamsNT.rotationKd.getValue());
        }

        // get current state
        poseWorldRobot = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
        velocityRobot = RobotStateRecorder.getVelocityWorldRobotCurrent();

        // calculate destination
        tagPose = useSelectedTarget ? AimGoalSupplier.getSelectedTag() : AimGoalSupplier.getNearestTag(poseWorldRobot);


        rightReef = DestinationSupplier.getInstance().isCoralRight();
        finalDestinationPose = AimGoalSupplier.getFinalCoralTarget(tagPose, rightReef);

        // Now that finalDestinationPose is set, we can get the drive target
        poseWorldTarget = AimGoalSupplier.getDriveTarget(poseWorldRobot, finalDestinationPose);

        // PID init with field-relative velocities
        rotationController.enableContinuousInput(0, Math.PI * 2);
        translationController.reset();
        rotationController.reset();
        indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMING);
    }

    @Override
    public void execute() {
        if (useFast()) {
            Logging.warn(kTag, "Use Fast!");
        } else {
            Logging.warn(kTag, "Use Slow!");
        }
        poseWorldRobot = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
        poseWorldTarget = AimGoalSupplier.getDriveTarget(poseWorldRobot, finalDestinationPose);
        Pose2d poseRobotTarget = poseWorldTarget.relativeTo(poseWorldRobot);
        velocityRobot = RobotStateRecorder.getVelocityRobotCurrent();

        // compute translation error, tu
        Translation2d pRT = poseRobotTarget.getTranslation();
        double pRT_norm = pRT.getNorm();
        Rotation2d pRT_dir = MathTools.toAngle(pRT);
        // NOTE: as pRT_norm is always positive, then vRT_norm is always negative.
        // to make the robot move along but not opposite to pRT_dir, we take the minus sign before vRT_norm
        double vRT_norm = -translationController.calculate(pRT_norm, 0.0);

        // compute rotation err, turn into angular velocity scalar
        double thetaRTOriginal = poseRobotTarget.getRotation().getRadians();
        double thetaRTAdjusted = poseRobotTarget.getTranslation().getAngle().getRadians();
        double maxThetaAdjustment = degreesToRadians(ReefAimCommandParamsNT.rotationAdjustmentMaxDegree.getValue());
        thetaRTAdjusted = MathUtil.clamp(thetaRTAdjusted, thetaRTOriginal - maxThetaAdjustment, thetaRTOriginal + maxThetaAdjustment);
        double omegaRT = -rotationController.calculate(thetaRTAdjusted, 0.0);

        // set limit
        double dCurr = finalDestinationPose.relativeTo(poseWorldRobot).getTranslation().getNorm(); // use final destination
        double vFar, vNear, dChange;
        if (useFast()) {
            vFar = ReefAimCommandParamsNT.translationFastVelocityMaxFar.getValue();
            vNear = ReefAimCommandParamsNT.translationFastVelocityMaxNear.getValue();
            dChange = ReefAimCommandParamsNT.translationFastParamsChangeDistance.getValue();
        } else {
            vFar = ReefAimCommandParamsNT.translationVelocityMaxFar.getValue();
            vNear = ReefAimCommandParamsNT.translationVelocityMaxNear.getValue();
            dChange = ReefAimCommandParamsNT.translationParamsChangeDistance.getValue();
        }

        double maxTranslationVelocityMps = dCurr > dChange ? vFar : vNear + dCurr / dChange * (vFar - vNear);
        vRT_norm = MathUtil.clamp(vRT_norm, 0.0, maxTranslationVelocityMps);
        Translation2d vRT = new Translation2d(vRT_norm, pRT_dir);

        // compose and run velocity with limit
        swerve.setSwerveLimit(
                SwerveLimit.builder()
                        .maxLinearVelocity(MetersPerSecond.of(maxTranslationVelocityMps))
                        .maxSkidAcceleration(MetersPerSecondPerSecond.of(ReefAimCommandParamsNT.translationAccelerationMax.getValue()))
                        .maxAngularVelocity(DegreesPerSecond.of(ReefAimCommandParamsNT.rotationVelocityMax.getValue()))
                        .maxAngularAcceleration(DegreesPerSecondPerSecond.of(ReefAimCommandParamsNT.rotationAccelerationMax.getValue()))
                        .build()
        );
        ChassisSpeeds VRT = new ChassisSpeeds(vRT.getX(), vRT.getY(), omegaRT);
        swerve.runTwist(VRT);

        // logging
        Logger.recordOutput(kTag + "/tagPose", tagPose);
        Logger.recordOutput(kTag + "/destinationPose", poseWorldTarget);
        Logger.recordOutput(kTag + "/finalDestinationPose", finalDestinationPose);
    }

    @Override
    public boolean isFinished() {
        Pose2d poseRobotTarget = poseWorldTarget.relativeTo(poseWorldRobot);
        Rotation3d imuRotation = swerve.getEstimatedPose().getRotation();
        if (useFast()) {
            xOnTarget = epsilonEquals(
                    poseRobotTarget.getTranslation().getX(), 0.0,
                    ReefAimCommandParamsNT.yOnTargetFastMeter.getValue()
            );
            yOnTarget = epsilonEquals(
                    poseRobotTarget.getTranslation().getY(), 0.0,
                    ReefAimCommandParamsNT.yOnTargetFastMeter.getValue()
            );
            xStationary = epsilonEquals(
                    velocityRobot.getTranslation().getX(), 0.0,
                    ReefAimCommandParamsNT.xStationaryFastMetersPerSecond.getValue()
            );
            yStationary = epsilonEquals(
                    velocityRobot.getTranslation().getY(), 0.0,
                    ReefAimCommandParamsNT.yStationaryFastMetersPerSecond.getValue()
            );
        } else {
            xOnTarget = epsilonEquals(
                    poseRobotTarget.getTranslation().getX(), 0.0,
                    ReefAimCommandParamsNT.xOnTargetMeter.getValue()
            );
            yOnTarget = epsilonEquals(
                    poseRobotTarget.getTranslation().getY(), 0.0,
                    ReefAimCommandParamsNT.yOnTargetMeter.getValue()
            );
            xStationary = epsilonEquals(
                    velocityRobot.getTranslation().getX(), 0.0,
                    ReefAimCommandParamsNT.xStationaryMetersPerSecond.getValue()
            );
            yStationary = epsilonEquals(
                    velocityRobot.getTranslation().getY(), 0.0,
                    ReefAimCommandParamsNT.yStationaryMetersPerSecond.getValue()
            );
        }


        rotationOnTarget = epsilonEquals(
                poseRobotTarget.getRotation().getDegrees(),
                0.0,
                ReefAimCommandParamsNT.rotationOnTargetToleranceDegree.getValue()
        );
        rotationStationary = epsilonEquals(
                velocityRobot.getRotation().getDegrees(), 0.0,
                ReefAimCommandParamsNT.rotationOnTargetVelocityToleranceDegreesPerSecond.getValue()
        );
        imuPitchStable = epsilonEquals(
                imuRotation.getMeasureY().in(Degree), 0.0,
                ReefAimCommandParamsNT.imuStationaryDeg.getValue()
        );
        imuRollStable = epsilonEquals(
                imuRotation.getMeasureX().in(Degree), 0.0,
                ReefAimCommandParamsNT.imuStationaryDeg.getValue()
        );


        Logger.recordOutput(kTag + "/xOnTarget", xOnTarget);
        Logger.recordOutput(kTag + "/xStationary", xStationary);
        Logger.recordOutput(kTag + "/yOnTarget", yOnTarget);
        Logger.recordOutput(kTag + "/yStationary", yStationary);
        Logger.recordOutput(kTag + "/rotationOnTarget", rotationOnTarget);
        Logger.recordOutput(kTag + "/rotationStationary", rotationStationary);
        Logger.recordOutput(kTag + "/imuPitchStable", imuPitchStable);
        Logger.recordOutput(kTag + "/imuRollStable", imuRollStable);
        return (
                xOnTarget && xStationary && yOnTarget && yStationary && rotationOnTarget && rotationStationary
                        && imuPitchStable && imuRollStable
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setSwerveLimitDefault();
        swerve.runStop();
        if (!interrupted) indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMED);
        else indicatorSubsystem.setPattern(IndicatorIO.Patterns.NORMAL);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}