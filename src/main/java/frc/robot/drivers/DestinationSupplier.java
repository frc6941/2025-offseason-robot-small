package frc.robot.drivers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.FieldConstants.Reef;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class DestinationSupplier {
    private static DestinationSupplier instance;
    @Getter
    private elevatorSetpoint currentElevSetpointCoral = elevatorSetpoint.L2;
    @Getter
    private boolean coralRight = false;
    @Getter
    @Setter
    public boolean useSuperCycle = true;
    @Getter
    @Setter
    private int targetTagID = 0;
    private boolean useCoral = true;
    @Getter
    private elevatorSetpoint currentElevSetpointAlgae = elevatorSetpoint.P1;
    @Getter
    private boolean isAuto = true;

    public static DestinationSupplier getInstance() {
        if (instance == null) {
            instance = new DestinationSupplier();
        }
        return instance;
    }

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
                        NavToStationCommandParamsNT.ROBOT_TO_PIPE_METERS.getValue(),
                        Constants.ReefAimCommand.PIPE_TO_TAG.magnitude() * (rightReef ? 1 : -1)),
                new Rotation2d()));
        return goal;
    }


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
            double distance = FieldConstants.officialAprilTagType.getLayout().getTagPose(i).get().
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

    /**
     * Gets the nearest AprilTag pose to the robot's current position
     *
     * @param robotPose Current pose of the robot
     * @return Pose2d of the nearest AprilTag, accounting for edge cases and controller input
     */
    public static Pose2d getNearestTag(Pose2d robotPose) {
        return FieldConstants.officialAprilTagType.getLayout().getTagPose(getNearestTagID(robotPose)).get().toPose2d();
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
            double distance = FieldConstants.officialAprilTagType.getLayout().getTagPose(i).get().
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

    private static int solveEdgeCase(double controllerX, double controllerY, int minDistanceID, int secondMinDistanceID) {
        record TagCondition(int tagA, int tagB, char axis, int positiveResult, int negativeResult) {
        }
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
     * Updates the elevator setpoint for either coral or poke positions
     *
     * @param setpoint The desired elevator setpoint (L1-L4 for coral, P1-P2 for poke)
     */
    @Deprecated
    public void updateElevatorSetpoint(elevatorSetpoint setpoint) {
        switch (setpoint) {
            case L2, L3, L4:
                currentElevSetpointCoral = setpoint;
                Logger.recordOutput("DestinationSupplier/currentElevSetpointCoral", setpoint);
                SmartDashboard.putString("DestinationSupplier/currentElevSetpointCoral", setpoint.toString());
                break;
            case P1, P2:
                currentElevSetpointAlgae = setpoint;
                Logger.recordOutput("DestinationSupplier/currentElevSetpointPoke", setpoint);
                SmartDashboard.putString("DestinationSupplier/currentElevSetpointPoke", setpoint.toString());
                break;
            default:
                System.out.println("Unknown elevator setpoint: " + setpoint);
        }
    }

    /**
     * Gets the current elevator setpoint value in meters
     *
     * @param useCoral Whether to use coral scoring position (true) or poke position (false)
     * @return The elevator extension distance in meters
     */
    @Deprecated
    public double getElevatorSetpoint(boolean useCoral) {
        this.useCoral = useCoral;
        if (useCoral) {
            return switch (currentElevSetpointCoral) {
                case L2 -> ElevatorCommonNT.L2_EXTENSION_METERS.getValue();
                case L3 -> ElevatorCommonNT.L3_EXTENSION_METERS.getValue();
                case L4 -> ElevatorCommonNT.L4_EXTENSION_METERS.getValue();
                default -> ElevatorCommonNT.INTAKE_EXTENSION_METERS.getValue();
            };
        } else {
            return switch (currentElevSetpointAlgae) {
                case P1 -> ElevatorCommonNT.P1_EXTENSION_METERS.getValue();
                case P2 -> ElevatorCommonNT.P2_EXTENSION_METERS.getValue();
                default -> ElevatorCommonNT.P1_EXTENSION_METERS.getValue();
            };
        }
    }

    /**
     * Updates which reef branch to target
     *
     * @param coralRight When true, targets the right reef relative to the AprilTag when facing it
     *                   (i.e. when you are facing the tag, rightReef = true means the tag on your right is the target)
     */
    public void updateBranch(boolean coralRight) {
        this.coralRight = coralRight;
        Logger.recordOutput("DestinationSupplier/Pipe", coralRight);
        SmartDashboard.putString("DestinationSupplier/Pipe", coralRight ? "Right" : "Left");
    }
    @Deprecated
    public enum elevatorSetpoint {
        L2, L3, L4, P1, P2
    }

    public static boolean isSafeToRaise(Pose2d robotPose, boolean rightReef) {
        Pose2d tag = AimGoalSupplier.getNearestTag(robotPose);
        Pose2d goal = AimGoalSupplier.getFinalCoralTarget(tag, rightReef);
        return goal.getTranslation().getDistance(robotPose.getTranslation()) < ReefAimCommandParamsNT.RAISE_LIMIT_METERS.getValue();
    }

    public void setIsAuto(boolean isAuto) {
        this.isAuto = isAuto;
        Logger.recordOutput("DestinationSupplier/Auto", isAuto);
        SmartDashboard.putString("DestinationSupplier/Auto", isAuto ? "Auto" : "Manual");
    }
}
