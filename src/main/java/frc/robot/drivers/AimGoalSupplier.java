package frc.robot.drivers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Reef;
import frc.robot.NavToStationCommandParamsNT;
import frc.robot.RobotStateRecorder;
import lib.ntext.NTParameter;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class AimGoalSupplier {

    /**
     * Calculates the optimal drive target position based on the robot's current position and goal position
     *
     * @param robot The current pose (position and rotation) of the robot
     * @param goal  The target pose to drive towards
     * @return A modified goal pose that accounts for optimal approach positioning
     */
    public static Pose2d getDriveTarget(Pose2d robot, Pose2d goal) {
        Transform2d offset = new Transform2d(goal, new Pose2d(robot.getTranslation(), goal.getRotation()));
        double yDistance = Math.abs(offset.getY());
        double xDistance = Math.abs(offset.getX());
        double shiftXT =
                MathUtil.clamp(
                        (yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)),
                        0.0,
                        1.0);
        double shiftYT = MathUtil.clamp(yDistance <= 0.2 ? 0.0 : -offset.getX() / Reef.faceLength, 0.0, 1.0);

        if (shiftXT < AimParamsNT.ShiftingTerminate.getValue())
            shiftXT = 0.0;
        if (shiftYT < AimParamsNT.ShiftingTerminate.getValue())
            shiftYT = 0.0;
        goal = goal.transformBy(
                new Transform2d(
//                        Math.copySign(shiftXT * AimParamsNT.MaxDistanceReefLineup.getValue(), offset.getX()),
                        shiftXT * AimParamsNT.MaxDistanceReefLineup.getValue(),
                        Math.copySign(shiftYT * AimParamsNT.MaxDistanceReefLineup.getValue() * 0.8, offset.getY()),
                        new Rotation2d()));

        return goal;
    }

    /**
     * Calculates the final target position for coral scoring based on the tag pose
     *
     * @param goal      The initial goal pose
     * @param rightReef Whether to target the right reef relative to the AprilTag
     * @return Modified goal pose to tag pose accounting for coral scoring position
     */
    public static Pose2d getFinalCoralTarget(Pose2d goal, boolean rightReef) {
        goal = goal.transformBy(new Transform2d(
                new Translation2d(
                        AimParamsNT.RobotToPipeXMeters.getValue(),
                        Constants.ReefAimCommand.PIPE_TO_TAG.magnitude() * (rightReef ? 1 : -1) + AimParamsNT.RobotToPipeYMeters.getValue()
                ),
                Rotation2d.fromDegrees(180)));
        return goal;
    }

    /**
     * Calculates the final target position for algae scoring based on the tag pose
     *
     * @param goal The initial goal pose
     * @return Modified goal pose to tag pose accounting for algae scoring position
     */
    public static Pose2d getFinalAlgaeTarget(Pose2d goal) {
        goal = goal.transformBy(new Transform2d(
                new Translation2d(
                        NavToStationCommandParamsNT.ROBOT_TO_ALGAE_METERS.getValue(),
                        NavToStationCommandParamsNT.ALGAE_TO_TAG_METERS.getValue()),
                new Rotation2d()));
        return goal;
    }

    public static Pose2d getFinalNetTarget() {
        return AllianceFlipUtil.apply(new Pose2d(
                new Translation2d(FieldConstants.Barge.cageLineX - AimParamsNT.NetClearanceDistance.getValue(), 0.0),
                Rotation2d.kZero
        ));
    }

    /**
     * Gets the nearest AprilTag pose to the robot's current position
     *
     * @param robotPose Current pose of the robot
     * @return Pose2d of the nearest AprilTag, accounting for edge cases and controller input
     */
    public static Pose2d getNearestTag(Pose2d robotPose) {
        return FieldConstants.aprilTagType.getLayout().getTagPose(getNearestTagID(robotPose)).get().toPose2d();
    }

    /**
     * Checks if the robot is in a hexagonal danger zone around the reef and facing towards it
     *
     * @param robotPose              Current pose of the robot
     * @param dangerZoneRadius       Radius of the hexagonal danger zone around the reef center (in meters)
     * @param facingToleranceDegrees Maximum angle tolerance for considering the robot as "facing the reef" (in degrees)
     * @return true if robot is in hexagonal danger zone AND facing towards the reef
     */
    public static boolean isInHexagonalReefDangerZone(Pose2d robotPose, double dangerZoneRadius, double facingToleranceDegrees) {
        Translation2d reefCenter = AllianceFlipUtil.apply(Reef.center);
        // Check if robot is inside the hexagonal danger zone
        boolean isInHexagon = isPointInHexagon(robotPose.getTranslation(), reefCenter, dangerZoneRadius);
        Logger.recordOutput("DangerZone/IsInHexagon", isInHexagon);

        if (!isInHexagon) {
            return false;
        }

        // Check if robot is facing towards the reef center
        Translation2d robotToReef = reefCenter.minus(robotPose.getTranslation());
        Rotation2d angleToReef = new Rotation2d(robotToReef.getX(), robotToReef.getY());

        // Robot's front is opposite to its rotation, so add 180 degrees
        Rotation2d robotFrontDirection = robotPose.getRotation().plus(Rotation2d.fromDegrees(180));
        double angleDifference = Math.abs(robotFrontDirection.minus(angleToReef).getDegrees());

        // Normalize angle difference to [0, 180] range
        if (angleDifference > 180) {
            angleDifference = 360 - angleDifference;
        }

        boolean isFacingReef = angleDifference <= facingToleranceDegrees;
        Logger.recordOutput("DangerZone/IsFacingReef", isFacingReef);

        return isFacingReef;
    }

    /**
     * Checks if the robot is in a hexagonal danger zone around the reef and facing towards it
     * Uses default parameters for danger zone radius and facing tolerance
     *
     * @param robotPose Current pose of the robot
     * @return true if robot is in hexagonal danger zone AND facing towards the reef
     */
    public static boolean isInHexagonalReefDangerZone(Pose2d robotPose) {

        double defaultDangerZoneRadius = Reef.faceLength
                + NavToStationCommandParamsNT.HEXAGON_DANGER_ZONE_OFFSET.getValue()
                + NavToStationCommandParamsNT.ROBOT_TO_ALGAE_METERS.getValue();
        double defaultFacingTolerance = NavToStationCommandParamsNT.HEXAGON_DANGER_DEGREES.getValue(); // Default facing tolerance in degrees

        return isInHexagonalReefDangerZone(robotPose, defaultDangerZoneRadius, defaultFacingTolerance);
    }

    /**
     * Checks if a point is inside a regular hexagon using the ray casting algorithm
     *
     * @param point     The point to check
     * @param hexCenter Center of the hexagon
     * @param radius    Distance from center to any vertex of the hexagon
     * @return true if the point is inside the hexagon
     */
    private static boolean isPointInHexagon(Translation2d point, Translation2d hexCenter, double radius) {
        // Generate hexagon vertices - rotated 30 degrees so flat edge is on top (like a reef)
        Translation2d[] hexVertices = new Translation2d[6];
        Pose3d[] hexVertices3d = new Pose3d[6];
        double hexHeight = 0.5; // Height of danger zone vertices in meters

        for (int i = 0; i < 6; i++) {
            double angle = Math.PI / 6.0 + Math.PI / 3.0 * i; // Start at 30° for flat-topped hexagon
            double x = hexCenter.getX() + radius * Math.cos(angle);
            double y = hexCenter.getY() + radius * Math.sin(angle);
            hexVertices[i] = new Translation2d(x, y);

            // Create Pose3d vertex facing outward from the hexagon center
            Translation3d translation3d = new Translation3d(x, y, hexHeight);
            Rotation3d rotation3d = new Rotation3d(0, 0, angle + Math.PI / 2.0); // Rotate 90° so pose faces outward
            hexVertices3d[i] = new Pose3d(translation3d, rotation3d);
        }

        // Log to AdvantageKit for visualization
        Logger.recordOutput("DangerZone/HexVertices3d", hexVertices3d);
        Logger.recordOutput("DangerZone/HexCenter", new Pose3d(hexCenter.getX(), hexCenter.getY(), hexHeight, new Rotation3d()));
        Logger.recordOutput("DangerZone/Radius", radius);

        // Ray casting algorithm: count intersections with polygon edges
        int intersections = 0;
        for (int i = 0; i < 6; i++) {
            Translation2d vertex1 = hexVertices[i];
            Translation2d vertex2 = hexVertices[(i + 1) % 6];

            // Check if horizontal ray from point intersects with this edge
            boolean intersects = ((vertex1.getY() > point.getY()) != (vertex2.getY() > point.getY())) &&
                    (point.getX() < (vertex2.getX() - vertex1.getX()) * (point.getY() - vertex1.getY()) /
                            (vertex2.getY() - vertex1.getY()) + vertex1.getX());

            if (intersects) {
                intersections++;
            }
        }

        // Point is inside if there's an odd number of intersections
        return (intersections % 2) == 1;
    }

    /**
     * Gets the ID of the nearest AprilTag to the robot's current position
     *
     * @param robotPose Current pose of the robot
     * @return ID of the nearest AprilTag, accounting for edge cases and controller input
     */
    public static int getNearestTagID(Pose2d robotPose) {
        XboxController driverController = new XboxController(0);
        double ControllerX = driverController.getLeftX();
        double ControllerY = driverController.getLeftY();
        double minDistance = Double.MAX_VALUE;
        double secondMinDistance = Double.MAX_VALUE;
        int ReefTagMin = AllianceFlipUtil.shouldFlip() ? 6 : 17;
        int ReefTagMax = AllianceFlipUtil.shouldFlip() ? 11 : 22;
        int minDistanceID = ReefTagMin;
        int secondMinDistanceID = ReefTagMin;
        for (int i = ReefTagMin; i <= ReefTagMax; i++) {
            double distance = FieldConstants.aprilTagType.getLayout().getTagPose(i).get().
                    toPose2d().getTranslation().getDistance(robotPose.getTranslation());
            if (distance < secondMinDistance) {
                secondMinDistanceID = i;
                secondMinDistance = distance;
            }
            if (distance < minDistance) {
                secondMinDistanceID = minDistanceID;
                secondMinDistance = minDistance;
                minDistanceID = i;
                minDistance = distance;
            }
        }
//        if ((secondMinDistance - minDistance) < NavToStationCommandParamsNT.Edge_Case_Max_Delta.getValue() && (Math.abs(ControllerX) >= 0.05 || Math.abs(ControllerY) >= 0.05)) {
//            minDistanceID = solveEdgeCase(ControllerX, ControllerY, minDistanceID, secondMinDistanceID);
//        }
        return minDistanceID;
    }

    public static int solveEdgeCase(double controllerX, double controllerY, int minDistanceID, int secondMinDistanceID) {
        List<TagCondition> conditions = AllianceFlipUtil.shouldFlip() ?
                List.of(
                        new TagCondition(6, 11, 'Y', 6, 11),
                        new TagCondition(8, 9, 'Y', 8, 9),
                        new TagCondition(6, 7, 'X', 7, 6),
                        new TagCondition(7, 8, 'X', 8, 7),
                        new TagCondition(9, 10, 'X', 9, 10),
                        new TagCondition(10, 11, 'X', 10, 11)
                ) :
                List.of(
                        new TagCondition(20, 19, 'Y', 19, 20),
                        new TagCondition(17, 22, 'Y', 17, 22),
                        new TagCondition(17, 18, 'X', 17, 18),
                        new TagCondition(18, 19, 'X', 18, 19),
                        new TagCondition(21, 22, 'X', 22, 21),
                        new TagCondition(20, 21, 'X', 21, 20)
                );
        for (TagCondition condition : conditions) {
            if (correctTagPair(secondMinDistanceID, minDistanceID, condition.tagA(), condition.tagB())) {
                double value = condition.axis() == 'X' ? controllerX : controllerY;
                minDistanceID = value > 0 ? condition.positiveResult() : condition.negativeResult();
                break;
            }
        }
        return minDistanceID;
    }

    private static boolean correctTagPair(double tag1, double tag2, double wantedTag1, double wantedTag2) {
        return (tag1 == wantedTag1 && tag2 == wantedTag2) || (tag1 == wantedTag2 && tag2 == wantedTag1);
    }

    /**
     * Checks if the robot is in an edge case situation between two tags and logs relevant information
     *
     * @param robotPose Current pose of the robot
     */
    public static void isEdgeCase(Pose2d robotPose) {
        XboxController driverController = new XboxController(0);
        double ControllerX = driverController.getLeftX();
        double ControllerY = driverController.getLeftY();
        double minDistance = Double.MAX_VALUE;
        double secondMinDistance = Double.MAX_VALUE;
        int ReefTagMin = AllianceFlipUtil.shouldFlip() ? 6 : 17;
        int ReefTagMax = AllianceFlipUtil.shouldFlip() ? 11 : 22;
        int minDistanceID = ReefTagMin;
        int secondMinDistanceID = ReefTagMin;
        for (int i = ReefTagMin; i <= ReefTagMax; i++) {
            double distance = FieldConstants.aprilTagType.getLayout().getTagPose(i).get().
                    toPose2d().getTranslation().getDistance(robotPose.getTranslation());
            if (distance < secondMinDistance) {
                secondMinDistanceID = i;
                secondMinDistance = distance;
            }
            if (distance < minDistance) {
                secondMinDistanceID = minDistanceID;
                secondMinDistance = minDistance;
                minDistanceID = i;
                minDistance = distance;
            }
        }
        Logger.recordOutput("EdgeCase/DeltaDistance", secondMinDistance - minDistance);
        Logger.recordOutput("EdgeCase/ControllerX", ControllerX);
        Logger.recordOutput("EdgeCase/ControllerY", ControllerY);
        if ((secondMinDistance - minDistance) < NavToStationCommandParamsNT.Edge_Case_Max_Delta.getValue()) {
            Logger.recordOutput("EdgeCase/IsEdgeCase", true);
            if (Math.abs(ControllerX) >= 0.05 || Math.abs(ControllerY) >= 0.05) {
                minDistanceID = solveEdgeCase(ControllerX, ControllerY, minDistanceID, secondMinDistanceID);
            }
        } else {
            Logger.recordOutput("EdgeCase/IsEdgeCase", false);
        }
        Logger.recordOutput("EdgeCase/TargetChanged", minDistanceID == secondMinDistanceID);
    }


    @NTParameter(tableName = "Params/AimParams")
    private static class AimParams {
        static final double MaxDistanceReefLineup = 0.75;
        static final double RobotToPipeXMeters = 0.48;
        static final double RobotToPipeYMeters = 0.00;

        static final double RobotToAlgaeMeters = 0.4;
        static final double AlgaeToTagMeters = 0.2;
        static final double HexagonDangerZoneOffset = 0.3;
        static final double HexagonDangerDegrees = 45.0;
        static final double EdgeCaseMaxDelta = 0.2;
        static final double ShiftingTerminate = 0.2;
        static final double NetClearanceDistance = 1.15;
        static final double NetNearDistance = 1.60;
    }

    private record TagCondition(int tagA, int tagB, char axis, int positiveResult, int negativeResult) {
    }

    public static Pose2d getStationTarget(boolean useRight) {
        return AllianceFlipUtil.apply(useRight ? FieldConstants.CoralStation.rightCenterFace : FieldConstants.CoralStation.leftCenterFace);
    }

    public static boolean useRightStation() {
        return RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d().minus(AllianceFlipUtil.apply(FieldConstants.CoralStation.rightCenterFace)).getTranslation().getNorm()
                < RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d().minus(AllianceFlipUtil.apply(FieldConstants.CoralStation.leftCenterFace)).getTranslation().getNorm();
    }

    public static Pose2d getFinalStationTarget(Pose2d goal) {
        return goal.transformBy(new Transform2d(
                new Translation2d(
                        NavToStationCommandParamsNT.ROBOT_TO_STATION_METERS.getValue(),
                        0
                ),
                new Rotation2d()));
    }
} 