// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package lib.ironpulse.utils;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Hertz;

/**
 * Adapted from https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/main/src/main/java/org/littletonrobotics/frc2025/subsystems/drive/PhoenixOdometryThread.java
 * <p>
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class PhoenixSynchronizationThread extends Thread {
    private static boolean isCANFD = new CANBus("*").isNetworkFD();
    private final Lock signalsLock = new ReentrantLock(); // Prevents conflicts when registering signals
    private final Lock userLock;
    private final double frequency;
    private final List<DoubleSupplier> genericSignals = new ArrayList<>();
    private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
    private final List<Queue<Double>> genericQueues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();
    private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];

    public PhoenixSynchronizationThread(Lock lock, Frequency frequency) {
        setName("PhoenixOdometryThread");
        setDaemon(true);
        this.userLock = lock;
        this.frequency = frequency.in(Hertz);
    }

    @Override
    public void start() {
        if (phoenixQueues.size() > 0 || timestampQueues.size() > 0) {
            super.start();
        }
    }

    /**
     * Registers a Phoenix signal to be read from the thread.
     */
    public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        userLock.lock();
        try {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
            System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
            newSignals[phoenixSignals.length] = signal;
            phoenixSignals = newSignals;
            phoenixQueues.add(queue);
        } finally {
            signalsLock.unlock();
            userLock.unlock();
        }
        return queue;
    }

    /**
     * Registers a generic signal to be read from the thread.
     */
    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        signalsLock.lock();
        userLock.lock();
        try {
            genericSignals.add(signal);
            genericQueues.add(queue);
        } finally {
            signalsLock.unlock();
            userLock.unlock();
        }
        return queue;
    }

    /**
     * Returns a new queue that returns timestamp values for each sample.
     */
    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        userLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            userLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        // DO NOT COPY UNLESS YOU UNDERSTAND THE CONSEQUENCES
        // https://docs.advantagekit.org/getting-started/template-projects/spark-swerve-template#real-time-thread-priority
        Threads.setCurrentThreadPriority(true, 1);

        while (true) {
            // Wait for updates from all signals
            signalsLock.lock();
            try {
                if (isCANFD && phoenixSignals.length > 0) {
                    BaseStatusSignal.waitForAll(2.0 / frequency, phoenixSignals);
                } else {
                    // "waitForAll" does not support blocking on multiple signals with a bus
                    // that is not CAN FD, regardless of Pro licensing. No reasoning for this
                    // behavior is provided by the documentation.
                    Thread.sleep((long) (1000.0 / frequency));
                    if (phoenixSignals.length > 0) BaseStatusSignal.refreshAll(phoenixSignals);
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
            } finally {
                signalsLock.unlock();
            }

            // Save new data to queues
            userLock.lock();
            try {
                // Sample timestamp is current FPGA time minus average CAN latency
                //     Default timestamps from Phoenix are NOT compatible with
                //     FPGA timestamps, this solution is imperfect but close
                double timestamp = RobotController.getFPGATime() / 1e6;
                double totalLatency = 0.0;
                for (BaseStatusSignal signal : phoenixSignals) {
                    totalLatency += signal.getTimestamp().getLatency();
                }
                if (phoenixSignals.length > 0) {
                    timestamp -= totalLatency / phoenixSignals.length;
                }

                // Add new samples to queues
                for (int i = 0; i < phoenixSignals.length; i++) {
                    phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
                }
                for (int i = 0; i < genericSignals.size(); i++) {
                    genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
                }
                for (int i = 0; i < timestampQueues.size(); i++) {
                    timestampQueues.get(i).offer(timestamp);
                }
            } finally {
                userLock.unlock();
            }
        }
    }
} 