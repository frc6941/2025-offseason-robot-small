package frc.robot.Subsystems.ElevatorSubsystem;


import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElevatorNTParamsNT;
import frc.robot.Constants.Elevator;
import lombok.Getter;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
    @AutoLogOutput(key = "Elevator/currentFilterValue")
    public double currentFilterValue = 0.0;
    @Getter
    @AutoLogOutput(key = "Elevator/zeroing")
    public boolean zeroing = false;
    @Getter
    @AutoLogOutput(key = "Elevator/goalPosition")
    public double goalPosition = 0.0;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        if(true){
            io.setElevatorTarget(goalPosition);
        }
            
        Logger.processInputs("Elevator", inputs);
        if(ElevatorNTParamsNT.isAnyChanged()){
            updateSlot0Config();
        }
    }

    public void setGoal(ElevatorState state){
        goalPosition = state.expectedPosition;
    }

    public Command zeroElevator() {
        return Commands.startRun(
            () -> {
                zeroing = true;
            },
            () -> {
                if (RobotBase.isReal()) {
                    currentFilterValue = currentFilter.calculate(inputs.statorCurrentAmps);
                    if (currentFilterValue <= 40) {
                        io.setElevatorVoltage(-1);
                    }
                    if (currentFilterValue > 40) {
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

    public void updateSlot0Config(){
        double kp = ElevatorNTParamsNT.ElevatorGearing.kP.getValue();
        double ki = ElevatorNTParamsNT.ElevatorGearing.kI.getValue();
        double kd = ElevatorNTParamsNT.ElevatorGearing.kD.getValue();
        double ks = ElevatorNTParamsNT.ElevatorGearing.kS.getValue();
        double kv = ElevatorNTParamsNT.ElevatorGearing.kV.getValue();
        double ka = ElevatorNTParamsNT.ElevatorGearing.kA.getValue();
        double kg = ElevatorNTParamsNT.ElevatorGearing.kG.getValue();
        io.configureSlot0Config(kp, ki, kd, ks, kv, ka, kg);
    }
}
