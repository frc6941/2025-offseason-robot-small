package frc.robot;

/**
 * Helper class that provides hasChanged functionality for SwerveModuleParamsNT.
 * This wraps the generated SwerveModuleParamsNT class to add change detection
 * without modifying the generated file.
 */
public class SwerveModuleParamsHelper {
    // Cached values for change detection (similar to TunableNumber.hasChanged())
    private static double lastDriveKp = Double.NaN;
    private static double lastDriveKi = Double.NaN;
    private static double lastDriveKd = Double.NaN;
    private static double lastDriveKs = Double.NaN;
    private static double lastDriveKv = Double.NaN;
    private static double lastDriveKa = Double.NaN;
    private static boolean lastDriveIsBrake = true;
    
    private static double lastSteerKp = Double.NaN;
    private static double lastSteerKi = Double.NaN;
    private static double lastSteerKd = Double.NaN;
    private static double lastSteerKs = Double.NaN;
    private static boolean lastSteerIsBrake = true;

    // hasChanged methods (similar to TunableNumber.hasChanged())
    public static boolean driveKpHasChanged() {
        double current = SwerveModuleParamsNT.driveKp();
        if (current != lastDriveKp) {
            lastDriveKp = current;
            return true;
        }
        return false;
    }

    public static boolean driveKiHasChanged() {
        double current = SwerveModuleParamsNT.driveKi();
        if (current != lastDriveKi) {
            lastDriveKi = current;
            return true;
        }
        return false;
    }

    public static boolean driveKdHasChanged() {
        double current = SwerveModuleParamsNT.driveKd();
        if (current != lastDriveKd) {
            lastDriveKd = current;
            return true;
        }
        return false;
    }

    public static boolean driveKsHasChanged() {
        double current = SwerveModuleParamsNT.driveKs();
        if (current != lastDriveKs) {
            lastDriveKs = current;
            return true;
        }
        return false;
    }

    public static boolean driveKvHasChanged() {
        double current = SwerveModuleParamsNT.driveKv();
        if (current != lastDriveKv) {
            lastDriveKv = current;
            return true;
        }
        return false;
    }

    public static boolean driveKaHasChanged() {
        double current = SwerveModuleParamsNT.driveKa();
        if (current != lastDriveKa) {
            lastDriveKa = current;
            return true;
        }
        return false;
    }

    public static boolean driveIsBrakeHasChanged() {
        boolean current = SwerveModuleParamsNT.driveIsBrake();
        if (current != lastDriveIsBrake) {
            lastDriveIsBrake = current;
            return true;
        }
        return false;
    }

    public static boolean steerKpHasChanged() {
        double current = SwerveModuleParamsNT.steerKp();
        if (current != lastSteerKp) {
            lastSteerKp = current;
            return true;
        }
        return false;
    }

    public static boolean steerKiHasChanged() {
        double current = SwerveModuleParamsNT.steerKi();
        if (current != lastSteerKi) {
            lastSteerKi = current;
            return true;
        }
        return false;
    }

    public static boolean steerKdHasChanged() {
        double current = SwerveModuleParamsNT.steerKd();
        if (current != lastSteerKd) {
            lastSteerKd = current;
            return true;
        }
        return false;
    }

    public static boolean steerKsHasChanged() {
        double current = SwerveModuleParamsNT.steerKs();
        if (current != lastSteerKs) {
            lastSteerKs = current;
            return true;
        }
        return false;
    }

    public static boolean steerIsBrakeHasChanged() {
        boolean current = SwerveModuleParamsNT.steerIsBrake();
        if (current != lastSteerIsBrake) {
            lastSteerIsBrake = current;
            return true;
        }
        return false;
    }

    /**
     * Checks if any parameter has changed. Useful for determining if any config update is needed.
     * @return true if any parameter has changed since last check
     */
    public static boolean anyHasChanged() {
        return driveKpHasChanged() || driveKiHasChanged() || driveKdHasChanged() ||
               driveKsHasChanged() || driveKvHasChanged() || driveKaHasChanged() ||
               driveIsBrakeHasChanged() || steerKpHasChanged() || steerKiHasChanged() ||
               steerKdHasChanged() || steerKsHasChanged() || steerIsBrakeHasChanged();
    }
} 