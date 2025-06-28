package lib.ironpulse.swerve.sjtu6;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import lib.ironpulse.swerve.ImuIO;
import lib.ironpulse.utils.PhoenixSynchronizationThread;
import lib.ironpulse.utils.PhoenixUtils;

import java.util.ArrayDeque;
import java.util.Queue;
import java.util.concurrent.locks.ReentrantLock;

public class ImuIOPigeon implements ImuIO {
    // Use the SAME sync thread and lock as swerve modules to ensure synchronized sampling
    private static final ReentrantLock syncLock = SwerveModuleIOSJTU6.getSyncLock();
    private static PhoenixSynchronizationThread syncThread = SwerveModuleIOSJTU6.getSyncThread();
    
    private final Pigeon2 pigeon;
    private final SwerveSJTU6Config config;
    
    // Status signals
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawVelocity;
    private final StatusSignal<Angle> pitch;
    private final StatusSignal<AngularVelocity> pitchVelocity;
    private final StatusSignal<Angle> roll;
    private final StatusSignal<AngularVelocity> rollVelocity;
    
    // Odometry queues (only for yaw since that's what's needed for pose estimation)
    private Queue<Double> yawPositionQueue;
    private Queue<Double> timestampQueue;

    public ImuIOPigeon(SwerveSJTU6Config config) {
        this.config = config;
        
        System.out.println("ImuIOPigeon: Initializing Pigeon2 with ID " + config.pigeonId);
        
        // Get the shared sync thread from swerve modules (it should already be created by now)
        //TODO: consider maybe move this to module?
        syncThread = SwerveModuleIOSJTU6.getSyncThread();
        
        // Initialize Pigeon2
        pigeon = new Pigeon2(config.pigeonId, config.canivoreCanBusName);
        
        // Get status signals
        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();
        pitch = pigeon.getPitch();
        pitchVelocity = pigeon.getAngularVelocityYWorld();
        roll = pigeon.getRoll();
        rollVelocity = pigeon.getAngularVelocityXWorld();
        
        // Configure signal update frequencies
        configureSignalFrequencies();
        
        // Register signals with PhoenixUtils for automatic refresh (same as swerve modules)
        PhoenixUtils.registerSignals(
                true,
                yaw,
                yawVelocity,
                pitch,
                pitchVelocity,
                roll,
                rollVelocity
        );
        
        // Register yaw signal for odometry queue (same pattern as swerve modules)
        if (syncThread != null) {
            yawPositionQueue = syncThread.registerSignal(yaw.clone());
            timestampQueue = syncThread.makeTimestampQueue();
        }
        
        // Fallback queues if sync thread registration fails
        if (yawPositionQueue == null) {
            yawPositionQueue = new ArrayDeque<>();
        }
        if (timestampQueue == null) {
            timestampQueue = new ArrayDeque<>();
        }
        
        pigeon.clearStickyFaults();
        pigeon.optimizeBusUtilization();
        
        System.out.println("ImuIOPigeon: Pigeon2 initialized successfully");
    }
    
    private void configureSignalFrequencies() {
        // High priority signals for control (100Hz = 10ms)
        yaw.setUpdateFrequency(config.odometryFrequency);
        yawVelocity.setUpdateFrequency(config.odometryFrequency);
        
        // Medium priority signals for telemetry (50Hz = 20ms) 
        pitch.setUpdateFrequency(50.0);
        pitchVelocity.setUpdateFrequency(50.0);
        roll.setUpdateFrequency(50.0);
        rollVelocity.setUpdateFrequency(50.0);
    }
    
    // Note: No need for startSyncThread() method since we use the shared sync thread from SwerveModuleIOSJTU6

    @Override
    public void updateInputs(ImuIOInputs inputs) {
        // Phoenix utils handles refreshing automatically, so we just read values
        inputs.connected = BaseStatusSignal.isAllGood(yaw, yawVelocity, pitch, pitchVelocity, roll, rollVelocity);
        
        // Current positions and velocities
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = yawVelocity.getValue().in(Units.RadiansPerSecond);
        inputs.pitchPosition = Rotation2d.fromDegrees(pitch.getValueAsDouble());
        inputs.pitchVelocityRadPerSec = pitchVelocity.getValue().in(Units.RadiansPerSecond);
        inputs.rollPosition = Rotation2d.fromDegrees(roll.getValueAsDouble());
        inputs.rollVelocityRadPerSec = rollVelocity.getValue().in(Units.RadiansPerSecond);
        
        // Process odometry queues (same pattern as swerve modules)
        if (!yawPositionQueue.isEmpty() && !timestampQueue.isEmpty()) {
            // Convert queue data to arrays
            int queueSize = Math.min(yawPositionQueue.size(), timestampQueue.size());
            
            inputs.odometryYawTimestamps = new double[queueSize];
            inputs.odometryYawPositions = new Rotation2d[queueSize];
            inputs.odometryRotations = new Rotation3d[queueSize];
            
            for (int i = 0; i < queueSize; i++) {
                double timestamp = timestampQueue.poll();
                double yawDegrees = yawPositionQueue.poll();
                
                inputs.odometryYawTimestamps[i] = timestamp;
                inputs.odometryYawPositions[i] = Rotation2d.fromDegrees(yawDegrees);
                inputs.odometryRotations[i] = new Rotation3d(
                    inputs.rollPosition.getRadians(),
                    inputs.pitchPosition.getRadians(),
                    Rotation2d.fromDegrees(yawDegrees).getRadians()
                );
            }
            
            // Clear any remaining queue items to stay synchronized
            yawPositionQueue.clear();
            timestampQueue.clear();
        } else {
            // Fallback to current values if no queue data
            inputs.odometryYawTimestamps = new double[]{yaw.getTimestamp().getTime()};
            inputs.odometryYawPositions = new Rotation2d[]{inputs.yawPosition};
            inputs.odometryRotations = new Rotation3d[]{
                new Rotation3d(
                    inputs.rollPosition.getRadians(),
                    inputs.pitchPosition.getRadians(),
                    inputs.yawPosition.getRadians()
                )
            };
        }
    }
} 