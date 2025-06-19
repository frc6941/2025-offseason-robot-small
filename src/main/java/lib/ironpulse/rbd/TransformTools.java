package lib.ironpulse.rbd;

import edu.wpi.first.math.geometry.*;

public class TransformTools {
    public static Transform3d relativeTo(Transform3d target, Transform3d reference) {
        return reference.inverse().plus(target);
    }

    public static Translation2d relativeTo(Translation2d target, Transform3d reference) {
        Transform3d invRef = reference.inverse();
        Transform3d targetTransform = new Transform3d(
                new Translation3d(target.getX(), target.getY(), 0.0), new Rotation3d());
        return invRef.plus(targetTransform).getTranslation().toTranslation2d();
    }

    public static Rotation2d relativeTo(Rotation2d target, Transform3d reference) {
        Transform3d invRef = reference.inverse();
        Transform3d targetTransform = new Transform3d(
                new Translation3d(), new Rotation3d(0.0, 0.0, target.getRadians()));
        double relativeYaw = invRef.plus(targetTransform).getRotation().getZ();
        return new Rotation2d(relativeYaw);
    }

    public static Translation3d relativeTo(Translation3d target, Transform3d reference) {
        Transform3d invRef = reference.inverse();
        Transform3d targetTransform = new Transform3d(target, new Rotation3d());
        return invRef.plus(targetTransform).getTranslation();
    }

    public static Rotation3d relativeTo(Rotation3d target, Transform3d reference) {
        Transform3d invRef = reference.inverse();
        Transform3d targetTransform = new Transform3d(new Translation3d(), target);
        return invRef.plus(targetTransform).getRotation();
    }

    public static Pose2d relativeTo(Pose2d target, Transform3d reference) {
        Translation2d relTrans = relativeTo(target.getTranslation(), reference);
        Rotation2d relRot = relativeTo(target.getRotation(), reference);
        return new Pose2d(relTrans, relRot);
    }

    public static Pose3d relativeTo(Pose3d target, Transform3d reference) {
        Translation3d relTrans = relativeTo(target.getTranslation(), reference);
        Rotation3d relRot = relativeTo(target.getRotation(), reference);
        return new Pose3d(relTrans, relRot);
    }

    public static Twist2d relativeTo(Twist2d target, Transform3d reference) {
        Rotation2d refYaw = new Rotation2d(reference.inverse().getRotation().getZ());
        Translation2d vel = new Translation2d(target.dx, target.dy).rotateBy(refYaw.unaryMinus());
        return new Twist2d(vel.getX(), vel.getY(), target.dtheta);
    }

    public static Twist3d relativeTo(Twist3d target, Transform3d reference) {
        Rotation3d refRot = reference.inverse().getRotation();
        Translation3d lin = new Translation3d(target.dx, target.dy, target.dz).rotateBy(refRot);
        Translation3d angVec = new Translation3d(target.rx, target.ry, target.rz).rotateBy(refRot);
        return new Twist3d(lin.getX(), lin.getY(), lin.getZ(), angVec.getX(), angVec.getY(), angVec.getZ());
    }

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
