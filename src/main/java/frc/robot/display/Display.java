package frc.robot.display;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotStateRecorder;
import lombok.Setter;

public class Display {
    // Singleton instance of Display
    private static Display instance;
    FieldView fieldView;
    @Setter
    private Pose2d aimingTarget = new Pose2d();

    // Private constructor to prevent instantiation
    private Display() {
        fieldView = new FieldView();
    }

    // Returns the singleton instance of Display, creating it if necessary
    public static Display getInstance() {
        if (instance == null) {
            instance = new Display();
        }
        return instance;
    }

    public void update() {
        fieldView.update(
                RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d(),
                aimingTarget
        );
    }
}