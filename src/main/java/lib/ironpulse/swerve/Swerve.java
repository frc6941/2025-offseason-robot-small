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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.ironpulse.utils.LoggedTracer;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.units.Units.*;

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
  @Getter
  private Voltage previouslyAppliedVoltage;
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
    imuIOInputs.yawVelocityRadPerSecCmd = getChassisSpeeds().omegaRadiansPerSecond;
    imuIO.updateInputs(imuIOInputs);
    Logger.processInputs(config.name + "/IMU", imuIOInputs);
    modules.forEach(module -> {
      module.updateInputs();
      module.periodic();
    });

    // odom
    var swerveModulePositionsWithTime = getSampledModulePositions();
    var rotations = imuIOInputs.odometryRotations;
    var now = Timer.getTimestamp();
    for (int i = 0; i < swerveModulePositionsWithTime.size(); i++) {
      var positionWithTime = swerveModulePositionsWithTime.get(i);
      poseEstimator.updateWithTime(
          now, rotations[i], // FIXME: there's a discrepancy between Phoenix time and rio time. need to find the offset. this fix is temporary
          positionWithTime.getSecond()
      );
    }
    odometryLock.unlock();
    LoggedTracer.record(config.name + "/Inputs");

    // telemetry
    Logger.recordOutput(config.name + "/Mode", mode);
    Logger.recordOutput(config.name + "/ChassisSpeedCurr", getChassisSpeeds());
    Logger.recordOutput(config.name + "/SwerveModuleStateCurr", getModuleStates());
    Logger.recordOutput(config.name + "/SwerveModuleStateCmd", setpointCurr.moduleStates());
    Logger.recordOutput(config.name + "/ChassisSpeedCmd", setpointCurr.chassisSpeeds());
    Logger.recordOutput(config.name + "/SwerveEstimatorPose", poseEstimator.getEstimatedPosition());

    var limit = getSwerveLimit();
    Logger.recordOutput(config.name + "/Limit/MaxLinVelMps", limit.maxLinearVelocity().in(MetersPerSecond));
    Logger.recordOutput(config.name + "/Limit/MaxSkidAccMps2", limit.maxSkidAcceleration().in(MetersPerSecondPerSecond));
    Logger.recordOutput(config.name + "/Limit/MaxAngvelDegps", limit.maxAngularVelocity().in(DegreesPerSecond));
    Logger.recordOutput(config.name + "/Limit/MaxAngAccDegps2", limit.maxAngularAcceleration().in(DegreesPerSecondPerSecond));
  }

  // -------- Run -------

  /**
   * Run a twist for the swerve drive.
   *
   * @param VRT the desired twist. note twist is expressed under the robot frame. if want to use field oriented
   *            drive,need to do frame transform elsewhere before pass in the command.
   */
  public void runTwist(ChassisSpeeds VRT) {
    mode = MODE.VELOCITY;
    setpointCurr = setpointGenerator.generate(VRT, setpointCurr, config.dtS);

    for (int i = 0; i < config.moduleCount(); i++)
      modules.get(i).runState(setpointCurr.moduleStates()[i]);
  }

  public void runTwistWithTorque(ChassisSpeeds VRT, Current[] tau) {
    assert (tau.length == config.moduleCount());
    mode = MODE.VELOCITY;
    setpointCurr = setpointGenerator.generate(VRT, setpointCurr, config.dtS);

    for (int i = 0; i < config.moduleCount(); i++)
      modules.get(i).runState(setpointCurr.moduleStates()[i], tau[i]);
  }

  public void runVoltage(Voltage voltage) {
    mode = MODE.VOLTAGE;
    previouslyAppliedVoltage = voltage;
    for (int i = 0; i < config.moduleCount(); i++)
      modules.get(i).runDriveVoltage(voltage);
  }

  public void runStop() {
    runVoltage(Volt.of(0.0));
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

  public void resetEstimatedPose(Pose3d pose) {
    poseEstimator.resetPose(pose);
  }

  public Optional<Pose3d> getEstimatedPoseAt(Time time) {
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
