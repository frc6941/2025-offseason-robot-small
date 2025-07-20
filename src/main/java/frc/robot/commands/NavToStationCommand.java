package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.NavToStationCommandParamsNT;
import frc.robot.RobotStateRecorder;
import frc.robot.drivers.AimGoalSupplier;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import lib.ironpulse.math.MathTools;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveLimit;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.units.Units.*;
import static lib.ironpulse.math.MathTools.epsilonEquals;

public class NavToStationCommand extends Command {
    private final static String kTag = "Commands/NavToStationCommand";
    private final Swerve swerve;
    private final IndicatorSubsystem indicatorSubsystem;
    private boolean rightReef; // true if shooting right reef
    private boolean xOnTarget, xStationary, yOnTarget, yStationary, rotationOnTarget, rotationStationary, imuPitchStable, imuRollStable;
    private Pose2d poseWorldRobot, velocityRobot, tagPose, poseWorldTarget, finalDestinationPose;
    private PIDController translationController;
    private PIDController rotationController;
    private Pose2d targetStation;

    private final boolean useSelectedTarget;

    public NavToStationCommand(Swerve swerve, IndicatorSubsystem indicatorSubsystem, boolean useSelectedTarget) {
        this.indicatorSubsystem = indicatorSubsystem;
        this.swerve = swerve;
        this.useSelectedTarget = useSelectedTarget;

        translationController = new PIDController(
                NavToStationCommandParamsNT.translationKp.getValue(),
                NavToStationCommandParamsNT.translationKi.getValue(),
                NavToStationCommandParamsNT.translationKd.getValue()
        );
        rotationController = new PIDController(
                NavToStationCommandParamsNT.rotationKp.getValue(),
                NavToStationCommandParamsNT.rotationKi.getValue(),
                NavToStationCommandParamsNT.rotationKd.getValue()
        );
        addRequirements(swerve);
    }

    public NavToStationCommand(Swerve swerve, IndicatorSubsystem indicatorSubsystem) {
        this(swerve, indicatorSubsystem, false);
    }


    @Override
    public void initialize() {
        translationController.setP(NavToStationCommandParamsNT.translationKp.getValue());
        translationController.setI(NavToStationCommandParamsNT.translationKi.getValue());
        translationController.setIZone(NavToStationCommandParamsNT.translationKiZone.getValue());
        translationController.setD(NavToStationCommandParamsNT.translationKd.getValue());

        rotationController.setP(NavToStationCommandParamsNT.rotationKp.getValue());
        rotationController.setI(NavToStationCommandParamsNT.rotationKi.getValue());
        rotationController.setIZone(NavToStationCommandParamsNT.rotationKiZone.getValue());
        rotationController.setD(NavToStationCommandParamsNT.rotationKd.getValue());

        // get current state
        poseWorldRobot = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
        velocityRobot = RobotStateRecorder.getVelocityWorldRobotCurrent();

        targetStation = AimGoalSupplier.getStationTarget(AimGoalSupplier.useRightStation());
        finalDestinationPose = AimGoalSupplier.getFinalStationTarget(targetStation);

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
        double maxThetaAdjustment = degreesToRadians(NavToStationCommandParamsNT.rotationAdjustmentMaxDegree.getValue());
        thetaRTAdjusted = MathUtil.clamp(thetaRTAdjusted, thetaRTOriginal - maxThetaAdjustment, thetaRTOriginal + maxThetaAdjustment);
        double omegaRT = -rotationController.calculate(thetaRTAdjusted, 0.0);

        // set limit
        double dCurr = finalDestinationPose.relativeTo(poseWorldRobot).getTranslation().getNorm(); // use final destination
        double vFar, vNear, dChange;
        vFar = NavToStationCommandParamsNT.translationVelocityMaxFar.getValue();
        vNear = NavToStationCommandParamsNT.translationVelocityMaxNear.getValue();
        dChange = NavToStationCommandParamsNT.translationParamsChangeDistance.getValue();

        double maxTranslationVelocityMps = dCurr > dChange ? vFar : vNear + dCurr / dChange * (vFar - vNear);
        vRT_norm = MathUtil.clamp(vRT_norm, 0.0, maxTranslationVelocityMps);
        Translation2d vRT = new Translation2d(vRT_norm, pRT_dir);

        // compose and run velocity with limit
        swerve.setSwerveLimit(
                SwerveLimit.builder()
                        .maxLinearVelocity(MetersPerSecond.of(maxTranslationVelocityMps))
                        .maxSkidAcceleration(MetersPerSecondPerSecond.of(NavToStationCommandParamsNT.translationAccelerationMax.getValue()))
                        .maxAngularVelocity(DegreesPerSecond.of(NavToStationCommandParamsNT.rotationVelocityMax.getValue()))
                        .maxAngularAcceleration(DegreesPerSecondPerSecond.of(NavToStationCommandParamsNT.rotationAccelerationMax.getValue()))
                        .build()
        );
        ChassisSpeeds VRT = new ChassisSpeeds(vRT.getX(), vRT.getY(), omegaRT);
        swerve.runTwist(VRT);

        // logging
        Logger.recordOutput(kTag + "/tagPose", tagPose);
        Logger.recordOutput(kTag + "/destinationPose", poseWorldTarget);
        Logger.recordOutput(kTag + "/finalDestinationPose", finalDestinationPose);
        Logger.recordOutput(kTag + "/targetStation", targetStation);
    }

    @Override
    public boolean isFinished() {
        Pose2d poseRobotTarget = poseWorldTarget.relativeTo(poseWorldRobot);
        Rotation3d imuRotation = swerve.getEstimatedPose().getRotation();
        xOnTarget = epsilonEquals(
                poseRobotTarget.getTranslation().getX(), 0.0,
                NavToStationCommandParamsNT.yOnTargetMeter.getValue()
        );
        yOnTarget = epsilonEquals(
                poseRobotTarget.getTranslation().getY(), 0.0,
                NavToStationCommandParamsNT.yOnTargetMeter.getValue()
        );
        xStationary = epsilonEquals(
                velocityRobot.getTranslation().getX(), 0.0,
                NavToStationCommandParamsNT.xStationaryMetersPerSecond.getValue()
        );
        yStationary = epsilonEquals(
                velocityRobot.getTranslation().getY(), 0.0,
                NavToStationCommandParamsNT.yStationaryMetersPerSecond.getValue()
        );


        rotationOnTarget = epsilonEquals(
                poseRobotTarget.getRotation().getDegrees(),
                0.0,
                NavToStationCommandParamsNT.rotationOnTargetToleranceDegree.getValue()
        );
        rotationStationary = epsilonEquals(
                velocityRobot.getRotation().getDegrees(), 0.0,
                NavToStationCommandParamsNT.rotationOnTargetVelocityToleranceDegreesPerSecond.getValue()
        );
        imuPitchStable = epsilonEquals(
                imuRotation.getMeasureY().in(Degree), 0.0,
                NavToStationCommandParamsNT.imuStationaryDeg.getValue()
        );
        imuRollStable = epsilonEquals(
                imuRotation.getMeasureX().in(Degree), 0.0,
                NavToStationCommandParamsNT.imuStationaryDeg.getValue()
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
        else indicatorSubsystem.setNormal();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}