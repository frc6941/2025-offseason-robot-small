package lib.ironpulse.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.ironpulse.utils.LoggedTracer;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static frc.robot.Constants.Swerve.kSwerveTag;

public class Swerve extends SubsystemBase {
    // locks
    static final Lock odometryLock = new ReentrantLock();
    // config and io
    private final SwerveConfig config;
    private final List<SwerveModule> modules;
    private final ImuIO imuIO;

    // controller
    private final SwerveDriveKinematics kinematics;
    private final SwerveSetpointGenerator setpointGenerator;
    // precomputed
    private final List<Rotation2d> xLockAngles;
    private final ImuIOInputsAutoLogged imuIOInputs;
    private SwerveSetpoint setpointCurr;
    private MODE mode = MODE.VELOCITY;

    public Swerve(SwerveConfig swerveConfig, ImuIO imuIO, SwerveModuleIO... moduleIOs) {
        this.config = swerveConfig;
        this.modules = new ArrayList<>(moduleIOs.length);
        if (config.moduleCount() != moduleIOs.length) {
            throw new Error("Module count mismatch: " + config.moduleCount() + " vs " + moduleIOs.length);
        }

        // ios
        this.imuIO = imuIO;
        this.imuIOInputs = new ImuIOInputsAutoLogged();
        for (int i = 0; i < config.moduleConfigs.length; i++)
            this.modules.add(i, new SwerveModule(config, config.moduleConfigs[i], moduleIOs[i]));

        SwerveModuleState[] states = new SwerveModuleState[config.moduleCount()];
        for (int i = 0; i < config.moduleCount(); i++)
            states[i] = modules.get(i).getSwerveModuleState();


        // kinematics, limits, and setpoint generator
        kinematics = new SwerveDriveKinematics(config.moduleLocations());
        setpointGenerator = SwerveSetpointGenerator.builder().kinematics(kinematics).chassisLimit(
                config.defaultSwerveLimit).moduleLimit(config.defaultSwerveModuleLimit).build();
        setpointCurr = new SwerveSetpoint(new ChassisSpeeds(), states);

        // precompute
        var moduleLocations = config.moduleLocations();
        xLockAngles = new ArrayList<>(config.moduleCount());
        for (int i = 0; i < config.moduleCount(); i++)
            xLockAngles.add(i, moduleLocations[i].getAngle());

    }

    // ------- Core Methods -------
    @Override
    public void periodic() {
        // io updates
        odometryLock.lock();
        imuIO.updateInputs(imuIOInputs);
        Logger.processInputs(kSwerveTag + "/IMU", imuIOInputs);
        modules.forEach(SwerveModule::updateInputs);
        odometryLock.unlock();
        LoggedTracer.record(kSwerveTag + "/Inputs");

        // module periodic
        modules.forEach(SwerveModule::periodic);

        // module commands
        if (DriverStation.isDisabled()) modules.forEach(SwerveModule::runStop);

    }

    // -------- Run -------
    public void runVelocity(ChassisSpeeds des) {
        mode = MODE.VELOCITY;
        var curr = getChassisSpeeds();
        setpointCurr = setpointGenerator.generate(curr, des, setpointCurr, Constants.kDtS);
        Logger.recordOutput(kSwerveTag + "/Setpoint", setpointCurr);
        for (int i = 0; i < config.moduleCount(); i++)
            modules.get(i).runState(setpointCurr.moduleStates()[i]);
    }

    public void runVoltage(Voltage voltage) {
        mode = MODE.VOLTAGE;
        for (int i = 0; i < config.moduleCount(); i++)
            modules.get(i).runDriveVoltage(voltage);
    }

    public void runStop() {
        runVelocity(new ChassisSpeeds());
    }

    public void runStopAndLock() {
        kinematics.resetHeadings(xLockAngles.toArray(new Rotation2d[0]));
        runStop();
    }

    // ------- Getters -------
    private List<SwerveModuleState> getModuleStates() {
        List<SwerveModuleState> states = new ArrayList<>(modules.size());
        for (SwerveModule module : modules) {
            states.add(module.getSwerveModuleState());
        }
        return states;
    }

    private ChassisSpeeds getChassisSpeeds() {
        SwerveModuleState[] statesArray = getModuleStates().toArray(new SwerveModuleState[0]);
        return kinematics.toChassisSpeeds(statesArray);
    }


    // ------- Configurations -------
    public SwerveLimit getSwerveLimit() {
        return setpointGenerator.getChassisLimit();
    }

    public void setSwerveLimit(SwerveLimit limit) {
        setpointGenerator.setChassisLimit(limit);
    }

    public void setSwerveLimitDefault() {
        setpointGenerator.setChassisLimit(config.defaultSwerveLimit);
    }

    public SwerveModuleLimit getSwerveModuleLimit() {
        return setpointGenerator.getModuleLimit();
    }

    public void setSwerveModuleLimit(SwerveModuleLimit limit) {
        setpointGenerator.setModuleLimit(limit);
    }

    public void setSwerveModuleLimitDefault() {
        setpointGenerator.setModuleLimit(config.defaultSwerveModuleLimit);
    }

    public enum MODE {
        VELOCITY, VOLTAGE
    }

}
