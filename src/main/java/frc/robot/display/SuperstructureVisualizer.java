package frc.robot.display;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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
    private void logCoralPose3D() {
//        // Check if coral is detected in the intake
//        if (GamepieceTracker.getInstance().isIntakeHasCoral()) {
//            Pose3d robotPose = new Pose3d(Swerve.getInstance().getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp()));
//
//            // Create a rotation matrix for the intake angle
//            Rotation3d intakeRotation = new Rotation3d(0, intakeAngleRad, 0);
//
//            // Calculate the position of the coral at the end of the intake arm
//            Pose3d coralPosition = robotPose.transformBy(new Transform3d(INTAKE_CENTER.plus(new Translation3d(INTAKE_LENGTH / 2, 0, 0).rotateBy(intakeRotation)), intakeRotation));
//
//            // Log the Coral pose 3D
//            Logger.recordOutput("Superstructure/Coral/InakeCoral",
//                    coralPosition);
//        } else {
//            // If no coral is detected, log an empty pose
//            Logger.recordOutput("Superstructure/Coral/InakeCoral", new Pose3d());
//        }
//        Pose3d robotPose = RobotStateRecorder.getPoseWorldRobotCurrent();
//        if (GamepieceTracker.getInstance().isEndeffectorHasCoral()) {
//            // Calculate the position of the coral at the middle of the end effector arm coral
//            Pose3d coralPosition = robotPose.transformBy(new Transform3d(
//                    endEffectorPosition.plus(new Translation3d(-END_EFFECTOR_LENGTH_CORAL, 0, END_EFFECTOR_MOUNT_ARM_LENGTH).rotateBy(endEffectorRotation)),
//                    endEffectorRotation));
//            Logger.recordOutput("Superstructure/Coral/EECoral", coralPosition);
//        } else {
//            Logger.recordOutput("Superstructure/Coral/EECoral", new Pose3d());
//        }
    }

    private void updateVisuals() {
        // Update elevator components
        elevatorHeight.setLength(currentElevatorHeight); // Stage 1 extends

        // Log Coral pose 3D
        logCoralPose3D();

        // Log 2D mechanisms
        Logger.recordOutput("Superstructure/Elevator/Mechanism2d", elevatorMechanism);
    }
}