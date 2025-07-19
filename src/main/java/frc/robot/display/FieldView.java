package frc.robot.display;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldView {
    private final Field2d mField2d = new Field2d();

    public FieldView() {
        SmartDashboard.putData(mField2d);
    }

    private void drawField() {
    }

    public void update(Pose2d pose, Pose2d aimingTarget) {
        drawField();
        mField2d.setRobotPose(pose);
        mField2d.getObject("Aiming Target").setPose(aimingTarget);
    }
}