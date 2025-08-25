package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.display.SuperstructureVisualizer;
import lib.ironpulse.utils.LoggedTracer;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Elevator.ELEVATOR_ZEROING_FILTER_SIZE;
import static frc.robot.ElevatorCommonNT.*;

public class ElevatorSubsystem extends SubsystemBase{
    @Getter
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final LinearFilter currentFilter = LinearFilter.movingAverage(ELEVATOR_ZEROING_FILTER_SIZE);
    @AutoLogOutput(key = "Elevator/currentFilterValue")
    public double currentFilterValue = 0.0;
    @Getter
    @AutoLogOutput(key = "Elevator/zeroing")
    public boolean zeroing = false;
    @Getter
    @AutoLogOutput(key = "Elevator/setPoint")
    private double wantedPosition = 0.16;
    private double previousWantedPosition = 0.16;
    @Getter
    @AutoLogOutput(key = "Elevator/atGoal")
    private boolean atGoal = false;
    @Getter
    @AutoLogOutput(key = "Elevator/isGoingUp")
    private boolean isGoingUp = false;
    @AutoLogOutput(key = "Elevator/runningCharacterization")
    private boolean runningCharacterization = false;
    @AutoLogOutput(key = "Elevator/stopDueToLimit")
    private boolean stopDueToLimit = false;

    // SysId routine for elevator characterization
    private final SysIdRoutine m_sysIdRoutine;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;

        // Initialize SysId routine after io is set
        this.m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Units.Volts.of(SYSID_RAMP_RATE_VOLTS_PER_SEC.getValue()).per(Units.Second), // Use default ramp rate (1 V/s) - can be adjusted via ElevatorConstants.SYSID_RAMP_RATE_VOLTS_PER_SEC
                        Units.Volts.of(SYSID_DYNAMIC_VOLTAGE.getValue()),
                        null, // Use default timeout (10 s) - can be adjusted via ElevatorConstants.SYSID_TIMEOUT_SECONDS
                        // Log state with Phoenix SignalLogger class
                        (state) -> SignalLogger.writeString("sysid-state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            io.setElevatorVoltage(volts.in(Units.Volts));
                            // Manually log the three required signals for SysId
                            SignalLogger.writeDouble("sysid-elevator-voltage", inputs.motorVoltage, "V");
                            SignalLogger.writeDouble("sysid-elevator-position", inputs.positionMeters, "m");
                            SignalLogger.writeDouble("sysid-elevator-velocity", inputs.velocityMetersPerSec, "m/s");
                        },
                        null, // No log consumer needed - using manual logging above
                        this
                )
        );
    }

    // SysId characterization commands

    /**
     * Returns a command that runs a quasistatic test in the given direction.
     *
     * @param direction The direction to run the test (kForward = up, kReverse = down)
     * @return The SysId quasistatic command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return Commands.sequence(
                Commands.runOnce(() -> runningCharacterization = true),
                m_sysIdRoutine.quasistatic(direction),
                Commands.runOnce(() -> runningCharacterization = false)
        );
    }

    /**
     * Returns a command that runs a dynamic test in the given direction.
     *
     * @param direction The direction to run the test (kForward = up, kReverse = down)
     * @return The SysId dynamic command
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return Commands.sequence(
                Commands.runOnce(() -> runningCharacterization = true),
                m_sysIdRoutine.dynamic(direction),
                Commands.runOnce(() -> runningCharacterization = false)
        );
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        SmartDashboard.putNumber("Elevator/PositionMeters", inputs.positionMeters);
        SmartDashboard.putNumber("Elevator/setPoint", wantedPosition);
        SmartDashboard.putBoolean("Elevator/zeroing", zeroing);

        // Check if position exceeds maximum extension
        if (wantedPosition > MAX_EXTENSION_METERS.getValue()) {
            stopDueToLimit = true;
            throw new IllegalArgumentException("Elevator setpoint " + wantedPosition + " exceeds maximum extension of " +
                    MAX_EXTENSION_METERS.getValue() + " meters");
        } else if (stopDueToLimit) {
            // Reset stopDueToLimit if position is now valid
            stopDueToLimit = false;
        }

        final boolean runningGoal = !stopDueToLimit && !zeroing && !runningCharacterization;
        if (runningGoal) {
            atGoal = elevatorAtGoal(ELEVATOR_GOAL_TOLERANCE.getValue());

            // Check if wanted position has changed to determine direction
            if (wantedPosition != previousWantedPosition) {
                isGoingUp = wantedPosition > previousWantedPosition;
//                System.out.println("Elevator direction changed: " + (isGoingUp ? "UP" : "DOWN") +
//                        " (from " + previousWantedPosition + " to " + wantedPosition + ")");
                previousWantedPosition = wantedPosition;
            }

            io.setElevatorTarget(wantedPosition, isGoingUp);
        } else {
            atGoal = false;
        }

        // Continuously log elevator signals during characterization
        if (runningCharacterization) {
            SignalLogger.writeDouble("elevator-motor-voltage", inputs.motorVoltage, "V");
            SignalLogger.writeDouble("elevator-position", inputs.positionMeters, "m");
            SignalLogger.writeDouble("elevator-velocity", inputs.velocityMetersPerSec, "m/s");
            SignalLogger.writeDouble("elevator-applied-volts", inputs.appliedVolts, "V");
            SignalLogger.writeDouble("elevator-stator-current", inputs.statorCurrentAmps, "A");
        }

        LoggedTracer.record("Elevator");

        SuperstructureVisualizer.getInstance().updateElevator(io.getElevatorHeight());
    }

    public double getElevatorPosition() {
        return inputs.positionMeters;
    }

    public void setElevatorPosition(DoubleSupplier position) {
        wantedPosition = position.getAsDouble();
    }

    public void setElevatorPosition(double position) {
        wantedPosition = position;
    }
    
    public boolean elevatorAtGoal(double offset) {
        return Math.abs(inputs.positionMeters - wantedPosition) < offset;
    }

    public Command zeroElevator() {
        return Commands.startRun(
                        () -> {
                            zeroing = true;
                        },
                        () -> {
                            if (RobotBase.isReal()) {
                                currentFilterValue = currentFilter.calculate(inputs.statorCurrentAmps);
                                if (currentFilterValue <= ELEVATOR_ZEROING_CURRENT.getValue()) {
                                    io.setElevatorVoltage(-1);
                                }
                                if (currentFilterValue > ELEVATOR_ZEROING_CURRENT.getValue()) {
                                    io.setElevatorVoltage(0);
                                    io.resetElevatorPosition();
                                    zeroing = false;
                                }
                            } else {
                                // In simulation, just set target to 0 (going down)
                                io.setElevatorTarget(0, false);
                                if (Math.abs(inputs.positionMeters) < 0.01) {
                                    zeroing = false;
                                }
                            }
                        })
                .until(() -> !zeroing)
                .finallyDo(() -> {
                    zeroing = false;
                });
    }

    public boolean isSafeToFlip() {
        return (inputs.positionMeters > SAFE_HEIGHT_FLIP.getValue());
    }
}