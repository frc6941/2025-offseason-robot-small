package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import lib.ironpulse.swerve.SwerveConfig;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ironpulse.swerve.SwerveModuleLimit;
import lib.ironpulse.swerve.sim.SwerveSimConfig;
import lib.ironpulse.swerve.sjtu6.SwerveSJTU6Config;
import lib.ironpulse.utils.Logging;
import lib.ntext.NTParameter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Ports.*;

public class Constants {
    /* -------------------------------------------------------------------------- */
    /*                               Global Settings                              */
    /* -------------------------------------------------------------------------- */
    public static final boolean kTuning = true;
    public static final double kDtS = 0.01;
    public static final String kParameterTag = "Params";
    public static final String CANIVORE_CAN_BUS_NAME = "10541Canivore0";
    public static final String RIO_CAN_BUS_NAME = "rio";
    public static final double LOOPER_DT = 0.02;
    public static final boolean TUNING = true;
    public static final boolean disableHAL = false;

    // auto robot config
    public static RobotConfig AUTO_ROBOT_CONFIG;

    static {
        try {
            AUTO_ROBOT_CONFIG = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            Logging.error("Constants", "Failed to load AUTO_ROBOT_CONFIG. %s", e.getMessage());
        }
    }


    /* -------------------------------------------------------------------------- */
    /*                               Swerve Settings                              */
    /* -------------------------------------------------------------------------- */
    public static final class Swerve {
        public static final String kSwerveTag = "Swerve";
        public static final String kSwerveModuleTag = "Swerve/SwerveModule";
        public static SwerveLimit kDefaultSwerveLimit = SwerveLimit.builder()
                .maxLinearVelocity(MetersPerSecond.of(4.5))
                .maxSkidAcceleration(MetersPerSecondPerSecond.of(10.0))
                .maxAngularVelocity(RadiansPerSecond.of(10.0))
                .maxAngularAcceleration(RadiansPerSecondPerSecond.of(15.0))
                .build();
        public static SwerveModuleLimit kDefaultSwerveModuleLimit = SwerveModuleLimit.builder()
                .maxDriveVelocity(MetersPerSecond.of(4.5))
                .maxDriveAcceleration(MetersPerSecondPerSecond.of(10.0))
                .maxSteerAngularVelocity(RadiansPerSecond.of(200.0))
                .maxSteerAngularAcceleration(RadiansPerSecondPerSecond.of(300.0))
                .build();
        public static SwerveConfig.SwerveModuleConfig kModuleCompFL = SwerveConfig.SwerveModuleConfig.builder()
                .name("FL")
                .location(new Translation2d(0.29, 0.29))
                .driveMotorId(SWERVE_FLD.id)
                .steerMotorId(SWERVE_FLS.id)
                .encoderId(SWERVE_FLC.id)
                .driveMotorEncoderOffset(Degree.of(50))
                .steerMotorEncoderOffset(Rotations.of(-0.4536))
                .driveInverted(false)
                .steerInverted(true)
                .encoderInverted(false)
                .build();
        public static SwerveConfig.SwerveModuleConfig kModuleCompFR = SwerveConfig.SwerveModuleConfig.builder()
                .name("FR")
                .location(new Translation2d(0.29, -0.29))
                .driveMotorId(SWERVE_FRD.id)
                .steerMotorId(SWERVE_FRS.id)
                .encoderId(SWERVE_FRC.id)
                .driveMotorEncoderOffset(Degree.of(50))
                .steerMotorEncoderOffset(Rotations.of(-0.419921875))
                .driveInverted(true)
                .steerInverted(true)
                .encoderInverted(false)
                .build();
        public static SwerveConfig.SwerveModuleConfig kModuleCompBL = SwerveConfig.SwerveModuleConfig.builder()
                .name("BL")
                .location(new Translation2d(-0.29, 0.29))
                .driveMotorId(SWERVE_BLD.id)
                .steerMotorId(SWERVE_BLS.id)
                .encoderId(SWERVE_BLC.id)
                .driveMotorEncoderOffset(Degree.of(50))
                .steerMotorEncoderOffset(Rotations.of(-0.347412109375))
                .driveInverted(false)
                .steerInverted(true)
                .encoderInverted(false)
                .build();
        public static SwerveConfig.SwerveModuleConfig kModuleCompBR = SwerveConfig.SwerveModuleConfig.builder()
                .name("BR")
                .location(new Translation2d(-0.29, -0.29))
                .driveMotorId(SWERVE_BRD.id)
                .steerMotorId(SWERVE_BRS.id)
                .encoderId(SWERVE_BRC.id)
                .driveMotorEncoderOffset(Degree.of(50))
                .steerMotorEncoderOffset(Rotations.of(0.4609375))
                .driveInverted(true)
                .steerInverted(true)
                .encoderInverted(false)
                .build();
        public static SwerveSimConfig kSimConfig = SwerveSimConfig.builder()
                .name("Swerve")
                .dtS(LOOPER_DT)
                .wheelDiameter(Inch.of(4.01))
                .driveGearRatio(7.0)
                .steerGearRatio(20.0)
                .driveMotor(DCMotor.getKrakenX60Foc(1))
                .driveMomentOfInertia(KilogramSquareMeters.of(0.025))
                .driveStdDevPos(0.0000001)
                .driveStdDevVel(0.000001)
                .steerMotor(DCMotor.getKrakenX60Foc(1))
                .steerMomentOfInertia(KilogramSquareMeters.of(0.004))
                .steerStdDevPos(0.0000001)
                .steerStdDevVel(0.000001)
                .defaultSwerveLimit(kDefaultSwerveLimit)
                .defaultSwerveModuleLimit(kDefaultSwerveModuleLimit)
                .moduleConfigs(new SwerveConfig.SwerveModuleConfig[]{
                        kModuleCompFL, kModuleCompFR, kModuleCompBL, kModuleCompBR
                })
                .build();
        public static SwerveSJTU6Config kRealConfig = SwerveSJTU6Config.builder()
                .name("Swerve")
                .dtS(LOOPER_DT)
                .wheelDiameter(Inch.of(4.01))
                .driveGearRatio(6.7460317460317460317460317460317)
                .steerGearRatio(21.428571428571428571428571428571)
                .defaultSwerveLimit(kDefaultSwerveLimit)
                .defaultSwerveModuleLimit(kDefaultSwerveModuleLimit)
                .moduleConfigs(new SwerveConfig.SwerveModuleConfig[]{
                        kModuleCompFL, kModuleCompFR, kModuleCompBL, kModuleCompBR
                })
                .odometryFrequency(Hertz.of(100))
                .driveStatorCurrentLimit(Amps.of(110))
                .steerStatorCurrentLimit(Amps.of(40))
                .canivoreCanBusName(CANIVORE_CAN_BUS_NAME)
                .pigeonId(PIGEON.id)
                .build();

        @NTParameter(tableName = kParameterTag + "/" + kSwerveModuleTag)
        private final static class SwerveModuleParams {
            private final static class Drive {
                static final double kP = 10;
                static final double kI = 0.16;
                static final double kD = 0.0;
                static final double kS = 0.0;
                static final double kV = 0.1247;
                static final double kA = 0.01215;
                static final boolean isBrake = true;
            }

            private final static class Steer {
                static final double kP = 10;
                static final double kI = 0.001;
                static final double kD = 0.15;
                static final double kS = 0.005;
                static final boolean isBrake = true;
            }
        }
    }

    /* -------------------------------------------------------------------------- */
    /*                              Controller Settings                           */
    /* -------------------------------------------------------------------------- */
    public static class Controller {
        public static final int kTest = 1;
        public static final int kDriver = 0;
    }

    /* -------------------------------------------------------------------------- */
    /*                              Elevator Settings                           */
    /* -------------------------------------------------------------------------- */
    public static class Elevator {
        public static final double ELEVATOR_SPOOL_DIAMETER = 0.04 + 0.003; //0.04m for spool diameter, 0.003 for rope diameter//TODO: change
        public static final double ELEVATOR_GEAR_RATIO = 3.0;//TODO: change
        public static final double ELEVATOR_DANGER_ZONE = 0.4180619200456253;//TODO: change
        public static final double ELEVATOR_DEFAULT_POSITION_WHEN_DISABLED = 0.0;//TODO: fixme
        public static final int ELEVATOR_ZEROING_FILTER_SIZE = 5;

        public static final String kTag = "Elevator";

        @NTParameter(tableName = kParameterTag + "/" + kTag)
        private static final class ElevatorCommon {
            private static final double ELEVATOR_GOAL_TOLERANCE = 0.02;

            private static final double MAX_EXTENSION_METERS = 1.54;
            private static final double ELEVATOR_ZEROING_CURRENT = 50.0;
            private static final double SAFE_HEIGHT_FLIP = 0.54;

            private static final double SYSID_RAMP_RATE_VOLTS_PER_SEC = 1;
            private static final double SYSID_DYNAMIC_VOLTAGE = 7;

            public static final double L2_EXTENSION_METERS = 0.494;
            public static final double L3_EXTENSION_METERS = 0.893;
            public static final double L4_EXTENSION_METERS = 1.535;
            public static final double P1_EXTENSION_METERS = 0.35;
            public static final double P2_EXTENSION_METERS = 0.75;
            public static final double INTAKE_EXTENSION_METERS = 0.0;

            public static final double motionAccelerationUp = 1000;
            public static final double motionCruiseVelocityUp = 1000;
            public static final double motionJerkUp = 0;

            public static final double motionAccelerationDown = 300;
            public static final double motionCruiseVelocityDown = 30;
            public static final double motionJerkDown = 0;

            private static class ElevatorGainsClass {
                private static final double ELEVATOR_KP = 1.67;
                private static final double ELEVATOR_KI = 0.00001;
                private static final double ELEVATOR_KD = 0.000010;
                private static final double ELEVATOR_KA = 0.000010;
                private static final double ELEVATOR_KV = 0.11;// 0.107853495
                private static final double ELEVATOR_KS = 0.8;
                private static final double ELEVATOR_KG = 0.420000;//0.3
            }
        }
    }

    public static final class EndEffector {

        public static final int MOTOR_ID = END_EFFECTOR.id;

        public static final int STATOR_CURRENT_LIMIT_AMPS = 80;
        public static final int SUPPLY_CURRENT_LIMIT_AMPS = 40;
        public static final boolean IS_BRAKE = true;
        public static final boolean IS_INVERT = false;
        public static final double ROLLER_RATIO = 1.0;//TODO: change

        public static final String kTag = "EndEffector";

        @NTParameter(tableName = kParameterTag + "/" + kTag)
        public static class EndEffectorParams {
            public static final double ROLLER_KP = 0;
            public static final double ROLLER_KI = 0;
            public static final double ROLLER_KD = 0;
            public static final double ROLLER_KA = 0;
            public static final double ROLLER_KV = 0;
            public static final double ROLLER_KS = 0;

            public static final double CORAL_INTAKE_VOLTAGE = 12.0;
            public static final double CORAL_INDEX_VOLTAGE = 4.0;
            public static final double CORAL_OUTTAKE_VOLTAGE = -6.0;
            public static final double CORAL_HOLD_VOLTAGE = 0.5;
            public static final double CORAL_SHOOT_VOLTAGE = 9.0;
            public static final double CORAL_SHOOT_DELAY_TIME = 0.2;
            public static final double ALGAE_POKE_VOLTAGE = 12.0;
            public static final double IDLE_VOLTAGE = 0.0d;
        }

    }

    public static class Indicator {
        public static final int LED_PORT = 0;//TODO: change
        public static final int LED_BUFFER_LENGTH = 30;//TODO: change
    }

    public static class Photonvision {
        public static final String[] PV_CAMERA_NAMES = {"LeftCamera", "RightCamera"};
        public static final boolean[] SNAPSHOT_ENABLED = {true, true};
        public static final int SNAPSHOT_PERIOD = 5; //seconds
        public static Transform3d[] CAMERA_RELATIVE_TO_ROBOT = new Transform3d[]{
                new Transform3d(0.19651247, 0.30981213, 0.3156, new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(-20))),
                new Transform3d(0.19651247, -0.30981213, 0.3156, new Rotation3d(Math.toRadians(0), Math.toRadians(0), Math.toRadians(20)))
        };
//        public static final String kPhotonVisionTag = "PhotonVision";
//        @NTParameter(tableName = "Params" + "/" + kPhotonVisionTag)
//        public final static class PhotonVisionParams {}
    }

    public static final class ReefAimCommand {
        public static final String kTag = "ReefAimConstants";

        public static final Measure<LinearVelocityUnit> MAX_AIMING_SPEED = MetersPerSecond.of(3.5);
        public static final Measure<LinearAccelerationUnit> MAX_AIMING_ACCELERATION = MetersPerSecondPerSecond.of(10);
        public static final Measure<DistanceUnit> PIPE_TO_TAG = Meters.of(0.164308503);

        @NTParameter(tableName = "Params/" + kTag)
        public static class ReefAimCommandParams {
            static final double translationKp = 2.5;
            static final double translationKi = 0.0;
            static final double translationKiZone = 0.00;
            static final double translationKd = 0.10;
            static final double translationVelocityMaxFar = 4.0;
            static final double translationVelocityMaxNear = 3.5;
            static final double translationParamsChangeDistance = 1.5;
            static final double translationAccelerationMax = 9.0;

            static final double translationFastKp = 3.0;
            static final double translationFastKi = 0.0;
            static final double translationFastKiZone = 0.00;
            static final double translationFastKd = 0.10;
            static final double translationFastVelocityMaxFar = 4.6;
            static final double translationFastVelocityMaxNear = 3.6;
            static final double translationFastParamsChangeDistance = 1.8;

            static final double rotationKp = 4.5;
            static final double rotationKi = 0.0;
            static final double rotationKiZone = 0.0;
            static final double rotationKd = 0.1;
            static final double rotationVelocityMax = 500.0;
            static final double rotationAccelerationMax = 2000.0;

            static final double xOnTargetFastMeter = 0.03;
            static final double yOnTargetFastMeter = 0.03;
            static final double xStationaryFastMetersPerSecond = 0.40;
            static final double yStationaryFastMetersPerSecond = 0.30;

            static final double imuStationaryDeg = 4.0;
            static final double rotationOnTargetToleranceDegree = 1.5;
            static final double rotationOnTargetVelocityToleranceDegreesPerSecond = 15.0;
            static final double rotationAdjustmentMaxDegree = 0.0;

            // old stuff

            public static final double HEXAGON_DANGER_ZONE_OFFSET = 0.24;
            public static final double MAX_DISTANCE_REEF_LINEUP = 0.75;
            public static final double ROBOT_TO_PIPE_METERS = 0.61;
            public static final double X_TOLERANCE_METERS = 0.01;
            public static final double Y_TOLERANCE_METERS = 0.01;
            public static final double RAISE_LIMIT_METERS = 1.0;
            public static final double OMEGA_TOLERANCE_DEGREES = 1;
            public static final double Edge_Case_Max_Delta = 0.3;
            public static final double ROBOT_TO_ALGAE_METERS = 0.489;
            public static final double ALGAE_TO_TAG_METERS = 0;
            public static final double HEXAGON_DANGER_DEGREES = 45;
        }

    }

    public static final class NavToStationCommand {
        public static final String kTag = "NavToStationCommand";

        public static final Measure<LinearVelocityUnit> MAX_AIMING_SPEED = MetersPerSecond.of(3.5);
        public static final Measure<LinearAccelerationUnit> MAX_AIMING_ACCELERATION = MetersPerSecondPerSecond.of(10);
        public static final Measure<DistanceUnit> PIPE_TO_TAG = Meters.of(0.164308503);

        @NTParameter(tableName = "Params/" + kTag)
        public static class NavToStationCommandParams {
            static final double translationKp = 3.2;
            static final double translationKi = 0.0;
            static final double translationKiZone = 0.00;
            static final double translationKd = 0.10;
            static final double translationVelocityMaxFar = 4.6;
            static final double translationVelocityMaxNear = 3.5;
            static final double translationParamsChangeDistance = 1.5;
            static final double translationAccelerationMax = 12.0;

            static final double rotationKp = 4.5;
            static final double rotationKi = 0.0;
            static final double rotationKiZone = 0.0;
            static final double rotationKd = 0.1;
            static final double rotationVelocityMax = 500.0;
            static final double rotationAccelerationMax = 2000.0;

            static final double xOnTargetMeter = 0.04;
            static final double yOnTargetMeter = 0.02;
            static final double xStationaryMetersPerSecond = 0.35;
            static final double yStationaryMetersPerSecond = 0.25;

            static final double imuStationaryDeg = 4.0;
            static final double rotationOnTargetToleranceDegree = 1.5;
            static final double rotationOnTargetVelocityToleranceDegreesPerSecond = 15.0;
            static final double rotationAdjustmentMaxDegree = 0.0;

            static final double ROBOT_TO_STATION_METERS = 0.47;
            // old stuff
            public static final double HEXAGON_DANGER_ZONE_OFFSET = 0.24;
            public static final double MAX_DISTANCE_REEF_LINEUP = 0.75;
            public static final double ROBOT_TO_PIPE_METERS = 0.59;
            public static final double X_TOLERANCE_METERS = 0.01;
            public static final double Y_TOLERANCE_METERS = 0.01;
            public static final double OMEGA_TOLERANCE_DEGREES = 1;
            public static final double Edge_Case_Max_Delta = 0.3;
            public static final double ROBOT_TO_ALGAE_METERS = 0.489;
            public static final double ALGAE_TO_TAG_METERS = 0;
            public static final double HEXAGON_DANGER_DEGREES = 45;
        }

    }
}

