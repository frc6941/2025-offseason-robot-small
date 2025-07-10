package frc.robot.subsystems.elevator;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.ironpulse.utils.LoggedTracer;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Elevator.ELEVATOR_ZEROING_FILTER_SIZE;
import static frc.robot.ElevatorCommonNT.*;

public class ElevatorSubsystem extends SubsystemBase {
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
    @Getter
    @AutoLogOutput(key = "Elevator/atGoal")
    private boolean atGoal = false;
    @AutoLogOutput(key = "Elevator/stopDueToLimit")
    private boolean stopDueToLimit = false;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        // Check if position exceeds maximum extension
        if (wantedPosition > MAX_EXTENSION_METERS.getValue()) {
            stopDueToLimit = true;
            throw new IllegalArgumentException("Elevator setpoint " + wantedPosition + " exceeds maximum extension of " +
                    MAX_EXTENSION_METERS.getValue() + " meters");
        } else if (stopDueToLimit) {
            // Reset stopDueToLimit if position is now valid
            stopDueToLimit = false;
        }

        final boolean runningGoal = !stopDueToLimit && !zeroing;
        if (runningGoal) {
            atGoal = elevatorAtGoal(ELEVATOR_GOAL_TOLERANCE.getValue());
            io.setElevatorTarget(wantedPosition);
        } else {
            atGoal = false;
        }
        LoggedTracer.record("Elevator");
    }

    public double getElevatorPosition() {
        return inputs.positionMeters;
    }

    public void setElevatorPosition(DoubleSupplier position) {
        wantedPosition = position.getAsDouble();
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
                                io.setElevatorTarget(0);
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
}