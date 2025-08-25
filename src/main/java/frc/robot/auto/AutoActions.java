package frc.robot.auto;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotStateRecorder;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.NavToStationCommand;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.ElevatorSetPoint;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SuperstructureState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import lib.ironpulse.rbd.TransformRecorder;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ntext.NTParameter;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Collections;
import java.util.List;

import static edu.wpi.first.units.Units.*;

public class AutoActions {
    private static final Pose2d kLeftIntakePoint = new Pose2d(
            new Translation2d(1.4, 6.8),
            Rotation2d.fromDegrees(144)
    );
    private static final Pose2d kLeftBackoff = new Pose2d(
            new Translation2d(2.7, 6.2),
            Rotation2d.fromDegrees(170)
    );
    private static final Pose2d kLeftEnd = new Pose2d(
            new Translation2d(2.50, 5.3),
            Rotation2d.fromDegrees(180)
    );
    private static final RotationTarget kLeftBackoffPointAngle = new RotationTarget(
            0.45, Rotation2d.fromDegrees(-10)
    );
    private static final RotationTarget kLeftIntakePointAngle = new RotationTarget(
            1.0, Rotation2d.fromDegrees(-26)
    );


    private static final Pose2d kRightIntakePoint = new Pose2d(
            new Translation2d(1.4, 1.2),
            Rotation2d.fromDegrees(-144)
    );
    private static final Pose2d kRightBackoff = new Pose2d(
            new Translation2d(2.7, 1.8),
            Rotation2d.fromDegrees(180.0)
    );
    private static final Pose2d kRightEnd = new Pose2d(
            new Translation2d(2.50, 2.7),
            Rotation2d.fromDegrees(180)
    );
    private static final RotationTarget kRightBackoffPointAngle = new RotationTarget(
            0.45, Rotation2d.fromDegrees(10)
    );
    private static final RotationTarget kRightIntakePointAngle = new RotationTarget(
            1.0, Rotation2d.fromDegrees(26)
    );

    public static Swerve swerve;
    public static IndicatorSubsystem indicator;
    public static ElevatorSubsystem elevatorSubsystem;
    public static EndEffectorSubsystem endEffectorSubsystem;
    private static Superstructure superstructure;

    public static void init(
            Swerve swerve,
            IndicatorSubsystem indicator,
            ElevatorSubsystem elevatorSubsystem,
            EndEffectorSubsystem endEffectorSubsystem,
            Superstructure superstructure) {
        AutoActions.swerve = swerve;
        AutoActions.indicator = indicator;
        AutoActions.elevatorSubsystem = elevatorSubsystem;
        AutoActions.endEffectorSubsystem = endEffectorSubsystem;
        AutoActions.superstructure = superstructure;
    }


    public static PathPlannerPath generatePath(List<Pose2d> waypoints, List<RotationTarget> rotationTargets, double maxVel, double maxAcc, double endVelMps) {
        PathConstraints constraints = new PathConstraints(
                maxVel, maxAcc,
                15.0, 40.0, 12.0
        );
        List<Waypoint> pts = PathPlannerPath.waypointsFromPoses(waypoints);
        Pose2d lastPose = waypoints.get(waypoints.size() - 1);
        GoalEndState endState = new GoalEndState(endVelMps, lastPose.getRotation());
        return new PathPlannerPath(
                pts,
                rotationTargets,
                Collections.emptyList(),
                Collections.emptyList(),
                Collections.emptyList(),
                constraints,
                null,
                endState,
                false
        );
    }

    public static PathPlannerPath generatePath(List<Pose2d> waypoints, List<RotationTarget> rotationTargets, double endVelMps) {
        return generatePath(waypoints, rotationTargets, 4.5, 15.0, endVelMps);
    }

    public static Command resetOnPathStart(PathPlannerPath path) {
        var realPath = AllianceFlipUtil.shouldFlip() ? path.flipPath() : path;

        return SwerveCommands.reset(swerve, new Pose3d(realPath.getStartingHolonomicPose().get()))
                .alongWith(Commands.runOnce(
                        () -> {
                            RobotStateRecorder.getInstance().resetTransform(
                                    TransformRecorder.kFrameWorld,
                                    TransformRecorder.kFrameRobot
                            );
                        }))
                .onlyIf(() -> realPath.getStartingHolonomicPose().isPresent())
                .ignoringDisable(true);
    }

    public static Command limitSwerve(
            double maxVelocityMps, double maxAccelerationMps2,
            double maxAngularVelDegps, double maxAngularAccelerationDegps2
    ) {
        return Commands.runOnce(() -> swerve.setSwerveLimit(
                SwerveLimit.builder()
                        .maxLinearVelocity(MetersPerSecond.of(maxVelocityMps))
                        .maxSkidAcceleration(MetersPerSecondPerSecond.of(maxAccelerationMps2))
                        .maxAngularVelocity(DegreesPerSecond.of(maxAngularVelDegps))
                        .maxAngularAcceleration(DegreesPerSecondPerSecond.of(maxAngularAccelerationDegps2))
                        .build()
        ));
    }

    public static Command unlimitSwerve() {
        return Commands.runOnce(swerve::setSwerveLimitDefault);
    }

    public static Command applySwerveLimit() {
        return Commands.run(() -> {
            double startLimitHeight = AutoParamsNT.TrajectoryLimitStartHeight.getValue();
            double maxVel = AutoParamsNT.TrajectoryMaxLinVelMps.getValue();
            double maxAcc = AutoParamsNT.TrajectoryMaxLinAccelMps2.getValue();
            double limitedVel, limitedAcc;
            limitedVel = maxVel;
            limitedAcc = maxAcc;
            swerve.setSwerveLimit(
                    SwerveLimit.builder()
                            .maxLinearVelocity(MetersPerSecond.of(limitedVel))
                            .maxSkidAcceleration(MetersPerSecondPerSecond.of(limitedAcc))
                            .maxAngularVelocity(DegreesPerSecond.of(
                                    AutoParamsNT.TrajectoryMaxAngVelDegps.getValue()
                            ))
                            .maxAngularAcceleration(DegreesPerSecondPerSecond.of(
                                    AutoParamsNT.TrajectoryMaxAngAccelDegps2.getValue()
                            ))
                            .build()
            );
        }).finallyDo(
                () -> swerve.setSwerveLimitDefault()
        );
    }

    public static Command followPath(PathPlannerPath path) {
        return new FollowPathCommand(
                path,
                () -> RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d(),
                swerve::getChassisSpeeds,
                (vel, ff) -> {
                    swerve.runTwist(vel);
                },
                new PPHolonomicDriveController(
                        new PIDConstants(5.5, 0.0, 0.1),
                        new PIDConstants(3.0, 0.0, 0.1),
                        Constants.LOOPER_DT
                ),
                Constants.AUTO_ROBOT_CONFIG,
                () -> false, // do not flip in command, flip done by user before passing
                swerve
        ).beforeStarting(
                () -> Logger.recordOutput("Temp/Traj", path.getPathPoses().toArray(new Pose2d[0]))
        );
    }

    public static Command autoScore(char goal, ElevatorSetPoint setPoint) {
        return new AutoShootCommand(swerve, indicator, elevatorSubsystem, endEffectorSubsystem, superstructure, setPoint, goal);
    }

    public static Command toStation(boolean isRight) {
        return new NavToStationCommand(swerve, indicator, isRight);
    }

    public static Command intake() {
        return new IntakeCommand(elevatorSubsystem, endEffectorSubsystem, indicator);
    }

    public static Command autoIntake(boolean isRight) {
        return Commands.sequence(
                Commands.runOnce(()->endEffectorSubsystem.setHasCoral(false)),
                superstructure.runGoal(SuperstructureState.IDLE).until(elevatorSubsystem::isAtGoal),
                new NavToStationCommand(swerve, indicator, isRight),
                superstructure.runGoal(SuperstructureState.INTAKE).until(endEffectorSubsystem::hasCoral)
        );
    }


    @NTParameter(tableName = "Params/Auto")
    public static class AutoParams {
        static final double TrajectoryMaxLinVelMps = 4.5;
        static final double TrajectoryMaxLinAccelMps2 = 14.0;
        static final double TrajectoryMaxAngVelDegps = 700.0;
        static final double TrajectoryMaxAngAccelDegps2 = 1800.0;

        static final double TrajectoryLimitedLinVelMps = 2.5;
        static final double TrajectoryLimitedLinAccelMps2 = 6.0;
        static final double TrajectoryLimitStartHeight = 0.65;

        static final double CoralInSightDegs = 80.0;

        static final double RightTriangleX = 2.5;
        static final double RightTriangleY = 1.7;
        static final double LeftTriangleX = 2.5;
        static final double LeftTriangleY = 6.3;
        static final double BoundaryOffset = 0.6;
    }

}
