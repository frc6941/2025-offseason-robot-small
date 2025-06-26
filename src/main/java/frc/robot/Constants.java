package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Frequency;
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
    public static enum RobotType {
        PRAC, COMP, SIM
    }
    public static final RobotType kRobotType = Robot.isReal()?RobotType.PRAC:RobotType.SIM;
    public static final boolean kTuning = true;
    public static final double kDtS = 0.01;
    public static final String kParameterTag = "Params";
    public static final String CANIVORE_CAN_BUS_NAME = "6941Canivore0";


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
                .steerMotorEncoderOffset(Rotations.of(-0.153564453125))
                .driveInverted(false)
                .steerInverted(false)
                .encoderInverted(true)
                .build();
        public static SwerveConfig.SwerveModuleConfig kModuleCompFR = SwerveConfig.SwerveModuleConfig.builder()
                .name("FR")
                .location(new Translation2d(0.5, -0.5))
                .driveMotorId(6)
                .steerMotorId(5)
                .encoderId(11)
                .driveMotorEncoderOffset(Degree.of(50))
                .steerMotorEncoderOffset(Rotations.of(-0.304443359375))
                .driveInverted(false)
                .steerInverted(false)
                .encoderInverted(true)
                .build();
        public static SwerveConfig.SwerveModuleConfig kModuleCompBL = SwerveConfig.SwerveModuleConfig.builder()
                .name("BL")
                .location(new Translation2d(-0.5, 0.5))
                .driveMotorId(2)
                .steerMotorId(1)
                .encoderId(0)
                .driveMotorEncoderOffset(Degree.of(50))
                .steerMotorEncoderOffset(Rotations.of(0.03955078125))
                .driveInverted(false)
                .steerInverted(false)
                .encoderInverted(true)
                .build();
        public static SwerveConfig.SwerveModuleConfig kModuleCompBR = SwerveConfig.SwerveModuleConfig.builder()
                .name("BR")
                .location(new Translation2d(-0.5, -0.5))
                .driveMotorId(8)
                .steerMotorId(7)
                .encoderId(20)
                .driveMotorEncoderOffset(Degree.of(50))
                .steerMotorEncoderOffset(Rotations.of(-0.379638671875))
                .driveInverted(false)
                .steerInverted(false)
                .encoderInverted(true)
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
                        kModuleCompFL,kModuleCompFR,kModuleCompBL,kModuleCompBR
                })
                .odometryFrequency(Hertz.of(50))
                .driveStatorCurrentLimit(Amps.of(110))
                .steerStatorCurrentLimit(Amps.of(110))
                .canivoreCanBusName(CANIVORE_CAN_BUS_NAME)
                .build();


        @NTParameter(tableName = kParameterTag + "/" + kSwerveModuleTag, isTuning = Constants.kTuning)
        private final static class SwerveModuleParams {
            static final double driveKp = 0.3;
            static final double driveKi = 0.0;
            static final double driveKd = 0.0;
            static final double driveKs = 0.0;
            static final double driveKv = 1.5;
            static final double driveKa = 0.0;
            static final boolean driveIsBrake = true;

            static final double steerKp = 120.0;
            static final double steerKi = 0.2;
            static final double steerKd = 0.005;
            static final boolean steerIsBrake = true;
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
}
