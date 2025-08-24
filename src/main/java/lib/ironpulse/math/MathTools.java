package lib.ironpulse.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Common utils.
 * GeomUtils are from Team 6328 Mechanical Advantage.
 */
public class MathTools {
    /**
     * Default tolerance for floating-point comparisons.
     */
    public static double TOLERANCE = 1e-5;

    /**
     * Normalize a vector safely, returning the original if its magnitude is below tolerance.
     *
     * @param input the vector to normalize.
     * @return a unit vector with same direction, or the original if too small.
     */
    public static Translation2d toNorm(Translation2d input) {
        double norm = input.getNorm();
        if (Math.abs(norm) < TOLERANCE) {
            return input;
        }
        return input.div(norm);
    }

    /**
     * Get the angle of a vector safely, return kZero if its magnitude is below tolerance.
     *
     * @param input the vector to take angle of.
     * @return an {@link Rotation2d} with the correct direction, or kZero if too small.
     */
    public static Rotation2d toAngle(Translation2d input) {
        double norm = input.getNorm();
        if (Math.abs(norm) < TOLERANCE) {
            return Rotation2d.kZero;
        }
        return input.getAngle();
    }

    /**
     * Clamp a vector's magnitude to a maximum value.
     *
     * @param input        the vector to clamp.
     * @param minMagnitude the minimum allowed magnitude.
     * @param maxMagnitude the maximum allowed magnitude.
     * @return the scaled vector if its length exceeds maxMagnitude, otherwise the original.
     */
    public static Translation2d clampMagnitude(Translation2d input, double minMagnitude, double maxMagnitude) {
        double norm = input.getNorm();
        double clamped = MathUtil.clamp(norm, minMagnitude, maxMagnitude);
        return input.div(norm == 0.0 ? 1.0 : norm).times(clamped);
    }

    /**
     * Clamp a vector's magnitude to a maximum value.
     *
     * @param input        the vector to clamp.
     * @param maxMagnitude the maximum allowed magnitude.
     * @return the scaled vector if its length exceeds maxMagnitude, otherwise the original.
     */
    public static Translation2d clampMagnitude(Translation2d input, double maxMagnitude) {
        return clampMagnitude(input, 0.0, maxMagnitude);
    }

    private double greaterInput(double input1, double input2) {
        if (Math.abs(input1) > Math.abs(input2)) return input1;
        return input2;
    }

    public static double clampMagnitude(double input, double maxMagnitude) {
        return MathUtil.clamp(Math.abs(input), 0.0, maxMagnitude) * Math.signum(input);
    }

    public static boolean epsilonEquals(double v1, double v2, double epsilon) {
        return Math.abs(v1 - v2) < epsilon;
    }

    public static boolean epsilonEquals(double v1, double v2) {
        return epsilonEquals(v1, v2, TOLERANCE);
    }

    public static boolean epsilonEquals(Translation2d v1, Translation2d v2, double epsilon) {
        return Math.abs(v1.getX() - v2.getX()) <= epsilon && Math.abs(v1.getY() - v2.getY()) <= epsilon;
    }

    public static boolean epsilonEquals(Translation2d v1, Translation2d v2) {
        return epsilonEquals(v1, v2, TOLERANCE);
    }

    // epsilon comparison for rotations
    public static boolean epsilonEquals(Rotation2d r1, Rotation2d r2, double epsilon) {
        double delta = r1.minus(r2).getRadians();
        return Math.abs(delta) <= epsilon;
    }

    public static boolean epsilonEquals(Rotation2d r1, Rotation2d r2) {
        return epsilonEquals(r1, r2, TOLERANCE);
    }

    public static boolean epsilonEquals(Twist2d t1, Twist2d t2, double epsilon) {
        return Math.abs(t1.dx - t2.dx) <= epsilon && Math.abs(t1.dy - t2.dy) <= epsilon && Math.abs(
                t1.dtheta - t2.dtheta) <= epsilon;
    }

    public static boolean epsilonEquals(Twist2d t1, Twist2d t2) {
        return epsilonEquals(t1, t2, TOLERANCE);
    }

    /**
     * Creates a pure translating transform
     *
     * @param translation The translation to create the transform with
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Translation2d translation) {
        return new Transform2d(translation, Rotation2d.kZero);
    }

    /**
     * Creates a pure translating transform
     *
     * @param x The x coordinate of the translation
     * @param y The y coordinate of the translation
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(double x, double y) {
        return new Transform2d(x, y, Rotation2d.kZero);
    }

    /**
     * Creates a pure rotating transform
     *
     * @param rotation The rotation to create the transform with
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Rotation2d rotation) {
        return new Transform2d(Translation2d.kZero, rotation);
    }

    /**
     * Converts a Pose2d to a Transform2d to be used in a kinematic chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    public static Transform2d toTransform2d(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }

    public static Pose2d inverse(Pose2d pose) {
        Rotation2d rotationInverse = pose.getRotation().unaryMinus();
        return new Pose2d(pose.getTranslation().unaryMinus().rotateBy(rotationInverse), rotationInverse);
    }

    /**
     * Converts a Transform2d to a Pose2d to be used as a position or as the start of a kinematic
     * chain
     *
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    public static Pose2d toPose2d(Transform2d transform) {
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }

    public static Pose2d toPose2d(ChassisSpeeds speed) {
        return new Pose2d(
                new Translation2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond),
                new Rotation2d(speed.omegaRadiansPerSecond)
        );
    }

    /**
     * Creates a pure translated pose
     *
     * @param translation The translation to create the pose with
     * @return The resulting pose
     */
    public static Pose2d toPose2d(Translation2d translation) {
        return new Pose2d(translation, Rotation2d.kZero);
    }

    /**
     * Creates a pure rotated pose
     *
     * @param rotation The rotation to create the pose with
     * @return The resulting pose
     */
    public static Pose2d toPose2d(Rotation2d rotation) {
        return new Pose2d(Translation2d.kZero, rotation);
    }

    /**
     * Multiplies a twist by a scaling factor
     *
     * @param twist  The twist to multiply
     * @param factor The scaling factor for the twist components
     * @return The new twist
     */
    public static Twist2d multiply(Twist2d twist, double factor) {
        return new Twist2d(twist.dx * factor, twist.dy * factor, twist.dtheta * factor);
    }

    /**
     * Converts a Pose3d to a Transform3d to be used in a kinematic chain
     *
     * @param pose The pose that will represent the transform
     * @return The resulting transform
     */
    public static Transform3d toTransform3d(Pose3d pose) {
        return new Transform3d(pose.getTranslation(), pose.getRotation());
    }

    /**
     * Converts a Transform3d to a Pose3d to be used as a position or as the start of a kinematic
     * chain
     *
     * @param transform The transform that will represent the pose
     * @return The resulting pose
     */
    public static Pose3d toPose3d(Transform3d transform) {
        return new Pose3d(transform.getTranslation(), transform.getRotation());
    }

    /**
     * Converts a ChassisSpeeds to a Twist2d by extracting two dimensions (Y and Z). chain
     *
     * @param speeds The original translation
     * @return The resulting translation
     */
    public static Twist2d toTwist2d(ChassisSpeeds speeds) {
        return new Twist2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    /**
     * Creates a new pose from an existing one using a different translation value.
     *
     * @param pose        The original pose
     * @param translation The new translation to use
     * @return The new pose with the new translation and original rotation
     */
    public static Pose2d withTranslation(Pose2d pose, Translation2d translation) {
        return new Pose2d(translation, pose.getRotation());
    }

    /**
     * Creates a new pose from an existing one using a different rotation value.
     *
     * @param pose     The original pose
     * @param rotation The new rotation to use
     * @return The new pose with the original translation and new rotation
     */
    public static Pose2d withRotation(Pose2d pose, Rotation2d rotation) {
        return new Pose2d(pose.getTranslation(), rotation);
    }


    /**
     * Unwrap an angle to be within ±π of a from.
     *
     * @param ref   the from angle (radians).
     * @param angle the angle to unwrap (radians).
     * @return the unwrapped angle closest to the from.
     */
    public static double unwrapAngle(double ref, double angle) {
        double diff = angle - ref;
        if (diff > Math.PI) {
            return angle - 2.0 * Math.PI;
        } else if (diff < -Math.PI) {
            return angle + 2.0 * Math.PI;
        }
        return angle;
    }

    public static double dot(Translation2d v1, Translation2d v2) {
        return v1.getX() * v2.getX() + v1.getY() * v2.getY();
    }

    public static double cross(Translation2d a, Translation2d b) {
        return a.getX() * b.getY() - a.getY() * b.getX();
    }
}
