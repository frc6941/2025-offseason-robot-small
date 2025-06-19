package lib.ironpulse.swerve.sim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import lib.ironpulse.swerve.SwerveModuleIO;

import static edu.wpi.first.units.Units.*;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private final SwerveSimConfig config;

    private DCMotorSim driveMotorSim;
    private double driveMotorAppliedVoltage;
    private DCMotorSim steerMotorSim;
    private double steerMotorAppliedVoltage;

    private PIDController driveFB;
    private boolean isDriveCloseLoop = false;
    private SimpleMotorFeedforward driveFF;
    private PIDController steerFB;
    private boolean isSteerCloseLoop = false;

    public SwerveModuleIOSim(SwerveSimConfig config) {
        this.config = config;
        initializePlants();
        initializeControllers();
    }

    private void initializePlants() {
        driveMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        config.driveMotor,
                        config.driveMomentOfInertia.in(KilogramSquareMeters),
                        config.driveGearRatio
                ), config.driveMotor, config.driveStdDevPos, config.driveStdDevVel
        );
        steerMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        config.steerMotor,
                        config.steerMomentOfInertia.in(KilogramSquareMeters),
                        config.steerGearRatio
                ), config.steerMotor, config.steerStdDevPos, config.steerStdDevVel
        );
    }

    private void initializeControllers() {
        driveFB = new PIDController(0.0, 0.0, 0.0);
        driveFF = new SimpleMotorFeedforward(0.0, 0.0);
        steerFB = new PIDController(0.0, 0.0, 0.0);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs data) {
        data.driveMotorConnected = true;
        data.driveMotorPositionRad = driveMotorSim.getAngularPositionRad();
        data.driveMotorPositionRadSamples = new double[]{data.driveMotorPositionRad};
        data.driveMotorVelocityRadPerSec = driveMotorSim.getAngularVelocityRadPerSec();
        data.driveMotorTemperatureCel = 25.0;
        data.driveMotorVoltageVolt = driveMotorAppliedVoltage;
        data.driveMotorSupplyCurrentAmpere = driveMotorSim.getCurrentDrawAmps();
        data.driveMotorTorqueCurrentAmpere = driveMotorSim.getTorqueNewtonMeters() / config.driveMotor.KtNMPerAmp;

        data.steerMotorConnected = true;
        data.steerMotorPositionRad = steerMotorSim.getAngularPositionRad();
        data.steerMotorPositionRadSamples = new double[]{data.steerMotorPositionRad};
        data.steerMotorVelocityRadPerSec = steerMotorSim.getAngularVelocityRadPerSec();
        data.steerMotorTemperatureCel = 25.0;
        data.steerMotorVoltageVolt = steerMotorAppliedVoltage;
        data.steerMotorSupplyCurrentAmpere = steerMotorSim.getCurrentDrawAmps();
        data.steerMotorTorqueCurrentAmpere = steerMotorSim.getTorqueNewtonMeters() / config.steerMotor.KtNMPerAmp;

        // step simulation
        driveMotorSim.setInputVoltage(data.driveMotorVoltageVolt);
        steerMotorSim.setInputVoltage(data.steerMotorVoltageVolt);
        driveMotorSim.update(Constants.kDtS);
        steerMotorSim.update(Constants.kDtS);
    }

    @Override
    public void setDriveOpenLoop(Voltage des) {
        isDriveCloseLoop = false;
        driveMotorAppliedVoltage = des.in(Volt);
    }

    @Override
    public void setDriveVelocity(LinearVelocity linearVelocityDes) {
        if (!isDriveCloseLoop) {
            isDriveCloseLoop = true;
            driveFB.reset();
        }
        Distance distancePerRotation = config.wheelDiameter.times(Math.PI);
        AngularVelocity angularVelocityDes = RotationsPerSecond.of(
                linearVelocityDes.in(MetersPerSecond) / distancePerRotation.in(Meter));

        double fb = driveFB.calculate(
                driveMotorSim.getAngularVelocityRadPerSec() / (2 * Math.PI), angularVelocityDes.in(RotationsPerSecond));
        double ff = driveFF.calculate(angularVelocityDes.in(RotationsPerSecond));
        driveMotorAppliedVoltage = fb + ff;
    }

    @Override
    public void setSteerOpenLoop(Voltage des) {
        isSteerCloseLoop = false;
        steerMotorAppliedVoltage = des.in(Volt);
    }

    @Override
    public void setSteerAngleAbsolute(Angle des) {
        if (!isSteerCloseLoop) {
            isSteerCloseLoop = true;
            steerFB.reset();
        }
        double fb = steerFB.calculate(Radian.of(steerMotorSim.getAngularPositionRad()).in(Radian), des.in(Radian));
        steerMotorAppliedVoltage = fb;
    }

    @Override
    public void configDriveKp(double kp) {
        driveFB.setP(kp);
    }

    @Override
    public void configDriveKi(double ki) {
        driveFB.setI(ki);
    }

    @Override
    public void configDriveKd(double kd) {
        driveFB.setD(kd);
    }

    @Override
    public void configDriveFF(SimpleMotorFeedforward ff) {
        driveFF = ff;
    }

    @Override
    public void configSteerKp(double kP) {
        steerFB.setP(kP);
    }

    @Override
    public void configSteerKi(double kI) {
        steerFB.setI(kI);
    }

    @Override
    public void configSteerKd(double kD) {
        steerFB.setD(kD);
    }

}
