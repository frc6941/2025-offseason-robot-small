package frc.robot.display;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Robot;
import frc.robot.RobotStateRecorder;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SuperstructureVisualizer {
    //todo: change the mechenism location to fit the real robot
    private static SuperstructureVisualizer instance;

    // Conversion helper
    private static double mmToM(double mm) {
        return mm / 1000.0;
    }

    // Elevator constants
    private static final Translation3d ELEVATOR_START = new Translation3d(
            mmToM(-200), mmToM(0), mmToM(90));
    private static final Translation3d ELEVATOR_END = new Translation3d(
            mmToM(-200), mmToM(0), mmToM(90));
    private static final Translation3d ELEVATOR_CENTER = ELEVATOR_START.interpolate(ELEVATOR_END, 0.5);

    // Stage specifications
    private static final double STAGE1_RETRACT_LENGTH = mmToM(958 + 8 + 8);
    private static final double STAGE1_TRAVEL = mmToM(790);
    private static final double STAGE2_RETRACTED_OFFSET = mmToM(18);
    private static final double STAGE2_TRAVEL = mmToM(820);
    private static final double STAGE2_LENGTH = mmToM(352);
    private static final double STAGE3_LENGTH = mmToM(314.29);

    private static final Translation3d END_EFFECTOR_CENTER = ELEVATOR_CENTER;
    private static final double END_EFFECTOR_LENGTH_CORAL = mmToM(170);
    private static final double END_EFFECTOR_LENGTH_ALGAE = mmToM(325);
    private static final double END_EFFECTOR_MOUNT_ARM_LENGTH = mmToM(242);


    // Visualization components
    private final LoggedMechanism2d elevatorMechanism;
    private final LoggedMechanismLigament2d elevatorHeight;
    private final LoggedMechanismLigament2d elevatorStage3;

    // Current state tracking
    private double currentElevatorHeight = 0.0;

    // Coral diameter in meters
    private static final double CORAL_DIAMETER = mmToM(100); // Adjust this value based on actual coral size


    public static SuperstructureVisualizer getInstance() {
        if (instance == null) {
            instance = new SuperstructureVisualizer();
        }
        return instance;
    }

    public SuperstructureVisualizer() {
        // Elevator mechanism setup
        elevatorMechanism = new LoggedMechanism2d(
                0,
                0,
                new Color8Bit(Color.kWhite));

        LoggedMechanismRoot2d elevatorRoot = elevatorMechanism.getRoot(
                "ElevatorBase",
                ELEVATOR_CENTER.getX(),
                ELEVATOR_CENTER.getZ());

        elevatorHeight = new LoggedMechanismLigament2d(
                "stage1",
                STAGE1_RETRACT_LENGTH,
                90,
                10,
                new Color8Bit(Color.kBlue));

        elevatorStage3 = new LoggedMechanismLigament2d(
                "elevatorStage3",
                STAGE3_LENGTH,
                0,
                8,
                new Color8Bit(Color.kBlue));
        elevatorRoot.append(elevatorHeight);
        elevatorHeight.append(elevatorStage3);
    }

    public void updateElevator(double elevatorHeight) {
        this.currentElevatorHeight = elevatorHeight;
        updateVisuals();
    }

    /**
     * Logs the Coral pose 3D if coral is detected in the intake
     */
    public void logCoralPose3D(boolean isEndeffectorHasCoral) {

        if (isEndeffectorHasCoral) {
            Pose3d robotPose = RobotStateRecorder.getPoseWorldRobotCurrent();

            // Calculate the position of the coral at the middle of the end effector arm
            // coral
            double endEffectorAngleRad = Math.toRadians(45);

            // Create a rotation matrix for the end effector angle
            Rotation3d endEffectorRotation = new Rotation3d(0, endEffectorAngleRad, 0);

            Translation3d endEffectorPosition = END_EFFECTOR_CENTER
                    .plus(new Translation3d(0, 0, currentElevatorHeight + STAGE3_LENGTH));

            // Calculate the position of the coral at the middle of the end effector arm
            // coral
            Pose3d coralPosition = robotPose.transformBy(new Transform3d(
                    endEffectorPosition
                            .plus(new Translation3d(-END_EFFECTOR_LENGTH_CORAL, 0, END_EFFECTOR_MOUNT_ARM_LENGTH)
                                    .rotateBy(endEffectorRotation)),
                    endEffectorRotation));

            Logger.recordOutput("Superstructure/Visualizer/Gamepiece/EECoral", coralPosition);
        } else {
            Logger.recordOutput("Superstructure/Visualizer/Gamepiece/EECoral", new Pose3d());
        }

    }

    private void updateVisuals() {
        elevatorHeight.setLength(currentElevatorHeight); // Stage 1 extends

        // Update intake components
//        intakeArm.setAngle(Rotation2d.fromRadians(Math.toRadians(-currentIntakeAngleDeg + 90)));

        // Update end effector components
//        endEffectorMountArm.setAngle(Rotation2d.fromRadians(Math.toRadians(currentEndEffectorAngleDeg + 180)));

        // Log 2D mechanisms
        if (Robot.isSimulation()) {
            Logger.recordOutput("Superstructure/Visualizer/" + "/Elevator/Mechanism2d", elevatorMechanism);
//            Logger.recordOutput("Superstructure/Visualizer/" + name + "/Intake/Mechanism2d", intakeMechanism);
        }
    }
    public void update(double elevatorHeight) {
        this.currentElevatorHeight = elevatorHeight;
        updateVisuals();
    }
}