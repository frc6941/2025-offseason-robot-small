package lib.ironpulse.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.ironpulse.utils.LoggedTracer;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.units.Units.Seconds;
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
    // estimator
    private final SwerveDrivePoseEstimator3d poseEstimator;
    // precomputed
    private final List<Rotation2d> xLockAngles;
    private final ImuIOInputsAutoLogged imuIOInputs;
    private SwerveSetpoint setpointCurr;
    private MODE mode = MODE.VELOCITY;

    public Swerve(SwerveConfig swerveConfig, ImuIO imuIO, SwerveModuleIO... moduleIOs) {
        this.config = swerveConfig;
        this.modules = new ArrayList<>(moduleIOs.length);
        if (config.moduleCount() != moduleIOs.length)
            throw new Error("Module count mismatch: " + config.moduleCount() + " vs " + moduleIOs.length);

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

        // estimator
        poseEstimator = new SwerveDrivePoseEstimator3d(
                kinematics,
                new Rotation3d(),
                getModulePositions(),
                new Pose3d()
        );

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

        // odom
        var swerveModulePositionsWithTime = getSampledModulePositions();
        var rotations = imuIOInputs.odometryRotations;
        for (int i = 0; i < swerveModulePositionsWithTime.size(); i++) {
            var positionWithTime = swerveModulePositionsWithTime.get(i);
            poseEstimator.updateWithTime(
                    positionWithTime.getFirst(), rotations[i],
                    positionWithTime.getSecond()
            );
        }
        odometryLock.unlock();
        LoggedTracer.record(kSwerveTag + "/Inputs");

        // module periodic
        modules.forEach((SwerveModule::periodic));
        if (DriverStation.isDisabled()) modules.forEach(SwerveModule::runStop);

        // telemetry
        Logger.recordOutput(kSwerveTag + "/Mode", mode);
        Logger.recordOutput(kSwerveTag + "/ChassisSpeedCurr", getChassisSpeeds());
        Logger.recordOutput(kSwerveTag + "/SwerveModuleStateCurr", getModuleStates());
        Logger.recordOutput(kSwerveTag + "/ChassisSpeedCmd", setpointCurr.chassisSpeeds());
        Logger.recordOutput(kSwerveTag + "/SwerveModuleStateCmd", setpointCurr.moduleStates());
        Logger.recordOutput(kSwerveTag + "/SwerveEstimatorPose", poseEstimator.getEstimatedPosition());
    }

    // -------- Run -------
    public void runVelocity(ChassisSpeeds des) {
        mode = MODE.VELOCITY;
        setpointCurr = setpointGenerator.generate(des, setpointCurr, Constants.kDtS);

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
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.size()];
        for (int i = 0; i < modules.size(); i++)
            states[i] = modules.get(i).getSwerveModuleState();
        return states;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[modules.size()];
        for (int i = 0; i < modules.size(); i++)
            states[i] = modules.get(i).getSwerveModulePosition();
        return states;
    }

    public List<Pair<Double, SwerveModulePosition[]>> getSampledModulePositions() {
        double[] timestamps = imuIOInputs.odometryYawTimestamps;
        int moduleCount = modules.size();

        // cache each module’s sampled positions array
        List<SwerveModulePosition[]> samplesByModule = modules.stream()
                .map(SwerveModule::getSampledSwerveModulePositions)
                .toList();

        List<Pair<Double, SwerveModulePosition[]>> result = new ArrayList<>(timestamps.length);
        for (int sampleIdx = 0; sampleIdx < timestamps.length; sampleIdx++) {
            // build the array of positions at this timestamp
            SwerveModulePosition[] positionsAtTime = new SwerveModulePosition[moduleCount];
            for (int moduleIdx = 0; moduleIdx < moduleCount; moduleIdx++)
                positionsAtTime[moduleIdx] = samplesByModule.get(moduleIdx)[sampleIdx];
            result.add(new Pair<>(timestamps[sampleIdx], positionsAtTime));
        }

        return result;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public Pose3d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Optional<Pose3d> getEstimatedPositionAt(Time time) {
        return poseEstimator.sampleAt(time.in(Seconds));
    }

    public void addVisionMeasurement(
            Pose3d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N4, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
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
