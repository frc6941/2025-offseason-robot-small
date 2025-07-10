package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import lib.ironpulse.swerve.SwerveConfig;
import lib.ironpulse.swerve.SwerveLimit;
import lib.ironpulse.swerve.SwerveModuleLimit;
import lib.ironpulse.swerve.sim.SwerveSimConfig;
import lib.ironpulse.swerve.sjtu6.SwerveSJTU6Config;
import lib.ntext.NTParameter;

import static edu.wpi.first.units.Units.*;

public class Constants {
    /* -------------------------------------------------------------------------- */
    /*                               Global Settings                              */
    /* -------------------------------------------------------------------------- */
    public static final boolean kTuning = true;
    public static final double kDtS = 0.01;
    public static final String kParameterTag = "Params";
    public static final String CANIVORE_CAN_BUS_NAME = "10541Canivore0";
    public static final String RIO_CAN_BUS_NAME = "rio";
    public static final int PIGEON_ID = 14;
    public static final double LOOPER_DT = 0.02;
    public static final boolean TUNING = true;


    /* -------------------------------------------------------------------------- */
    /*                               Swerve Settings                              */
    /* -------------------------------------------------------------------------- */
    public static final class Swerve {
        public static final String kSwerveTag = "Swerve";
        public static final String kSwerveModuleTag = "Swerve/SwerveModule";
        public static SwerveLimit kDefaultSwerveLimit = SwerveLimit.builder()
                .maxLinearVelocity(MetersPerSecond.of(4.5))
                .maxSkidAcceleration(MetersPerSecondPerSecond.of(22.0))
                .maxAngularVelocity(RadiansPerSecond.of(10.0))
                .maxAngularAcceleration(RadiansPerSecondPerSecond.of(15.0))
                .build();
        public static SwerveModuleLimit kDefaultSwerveModuleLimit = SwerveModuleLimit.builder()
                .maxDriveVelocity(MetersPerSecond.of(4.5))
                .maxDriveAcceleration(MetersPerSecondPerSecond.of(25.0))
                .maxSteerAngularVelocity(RadiansPerSecond.of(200.0))
                .maxSteerAngularAcceleration(RadiansPerSecondPerSecond.of(300.0))
                .build();
        public static SwerveConfig.SwerveModuleConfig kModuleCompFL = SwerveConfig.SwerveModuleConfig.builder()
                .name("FL")
                .location(new Translation2d(0.5, 0.5))
                .driveMotorId(4)
                .steerMotorId(3)
                .encoderId(10)
                .driveMotorEncoderOffset(Degree.of(50))
                .steerMotorEncoderOffset(Rotations.of(-0.4536))
                .driveInverted(true)
                .steerInverted(true)
                .encoderInverted(false)
                .build();
        public static SwerveConfig.SwerveModuleConfig kModuleCompFR = SwerveConfig.SwerveModuleConfig.builder()
                .name("FR")
                .location(new Translation2d(0.5, -0.5))
                .driveMotorId(6)
                .steerMotorId(5)
                .encoderId(11)
                .driveMotorEncoderOffset(Degree.of(50))
                .steerMotorEncoderOffset(Rotations.of(-0.419921875))
                .driveInverted(false)
                .steerInverted(true)
                .encoderInverted(false)
                .build();
        public static SwerveConfig.SwerveModuleConfig kModuleCompBL = SwerveConfig.SwerveModuleConfig.builder()
                .name("BL")
                .location(new Translation2d(-0.5, 0.5))
                .driveMotorId(2)
                .steerMotorId(1)
                .encoderId(0)
                .driveMotorEncoderOffset(Degree.of(50))
                .steerMotorEncoderOffset(Rotations.of(-0.347412109375))
                .driveInverted(true)
                .steerInverted(true)
                .encoderInverted(false)
                .build();
        public static SwerveConfig.SwerveModuleConfig kModuleCompBR = SwerveConfig.SwerveModuleConfig.builder()
                .name("BR")
                .location(new Translation2d(-0.5, -0.5))
                .driveMotorId(8)
                .steerMotorId(7)
                .encoderId(20)
                .driveMotorEncoderOffset(Degree.of(50))
                .steerMotorEncoderOffset(Rotations.of(0.4609375))
                .driveInverted(false)
                .steerInverted(true)
                .encoderInverted(false)
                .build();
        public static SwerveSimConfig kSimConfig = SwerveSimConfig.builder()
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
                .steerStatorCurrentLimit(Amps.of(110))
                .canivoreCanBusName(CANIVORE_CAN_BUS_NAME)
                .pigeonId(PIGEON_ID)
                .build();

        @NTParameter(tableName = kParameterTag + "/" + kSwerveModuleTag)
        private final static class SwerveModuleParams {
            private final static class Drive {
                static final double kP = 1;
                static final double kI = 0.0;
                static final double kD = 0.0;
                static final double kS = 0;
                static final double kV = 0;
                static final double kA = 0.0;
                static final boolean isBrake = true;
            }

            private final static class Steer {
                static final double kP = 10;
                static final double kI = 0;
                static final double kD = 0.1;
                static final double kS = 0.022;
                static final boolean isBrake = true;
            }
        }
    }

    /* -------------------------------------------------------------------------- */
    /*                              Controller Settings                           */
    /* -------------------------------------------------------------------------- */
    public static class Controller {
        public static final int kDriver = 0;
        public static final int kOperator = 1;
        public static final int kTest = 2;
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
            private static final double motionAcceleration = 300.0;
            private static final double motionCruiseVelocity = 100;

            private static final double motionJerk = 0.0;
            private static final double MAX_EXTENSION_METERS = 1.41;
            private static final double ELEVATOR_ZEROING_CURRENT = 40;
            private static final double SAFE_HEIGHT_FLIP = 0.54;

            private static class ElevatorGainsClass {
                private static final double ELEVATOR_KP = 2.5;
                private static final double ELEVATOR_KI = 0;
                private static final double ELEVATOR_KD = 0;
                private static final double ELEVATOR_KA = 0;
                private static final double ELEVATOR_KV = 0.08;// 0.107853495
                private static final double ELEVATOR_KS = 0.1;
                private static final double ELEVATOR_KG = 0.2;//0.3
            }

        }


    }

    public static final class EndEffector {

        public static final int MOTOR_ID = 22;

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
            public static final double CORAL_OUTTAKE_VOLTAGE = -6.0;
            public static final double CORAL_PRESHOOT_VOLTAGE = -10.0;
            public static final double CORAL_HOLD_VOLTAGE = 0.5;
            public static final double CORAL_SHOOT_VOLTAGE = -12.0;
            public static final double CORAL_SHOOT_VOLTAGE_L1 = -2.0;
            public static final double CORAL_SHOOT_DELAY_TIME = 0.2;
        }

    }

    public static class Limelight {
        public static final String LIMELIGHT_LEFT = "limelight-leftf";//TODO: change
        public static final String LIMELIGHT_RIGHT = "limelight-rightf";//TODO: change
        public static final double AREA_THRESHOLD = 0.1;
        public static final String kTag = "LimeLight";

        @NTParameter(tableName = kParameterTag + "/" + kTag)
        public static class LimeLightParams {
            public static final double OCULUS_RESET_AMBIGUITY_THRESHOLD = 0.15;
        }
    }

    public static class Indicator {
        public static final int LED_PORT = 0;//TODO: change
        public static final int LED_BUFFER_LENGTH = 30;//TODO: change
    }
}

