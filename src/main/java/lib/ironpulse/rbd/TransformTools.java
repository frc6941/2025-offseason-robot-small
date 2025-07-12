package lib.ironpulse.rbd;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class TransformTools {
    public static Pose3d inverse(Pose3d target) {
        var temp = new Transform3d(target.getTranslation(), target.getRotation()).inverse();
        return new Pose3d(temp.getTranslation(), temp.getRotation());
    }

    public static Transform3d toTransform3d(Pose3d target) {
        return new Transform3d(target.getTranslation(), target.getRotation());
    }

    public static Pose3d toPose3d(Transform3d target) {
        return new Pose3d(target.getTranslation(), target.getRotation());
    }
}
