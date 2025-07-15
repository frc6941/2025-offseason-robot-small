package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ElevatorCommonNT;
import frc.robot.EndEffectorParamsNT;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;

public class ManualShootCommand extends Command {
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final Timer shootTimer = new Timer();
    private boolean shootFinished = false;

    public ManualShootCommand(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevatorSubsystem.setElevatorPosition(DestinationSupplier.getInstance().getElevatorSetpoint(true));
        if (elevatorSubsystem.isAtGoal()) {
            endEffectorSubsystem.setRollerVoltage(EndEffectorParamsNT.CORAL_SHOOT_VOLTAGE.getValue());
            if (!endEffectorSubsystem.hasCoral()) {
                if (!shootTimer.isRunning()) shootTimer.start();
                if (shootTimer.hasElapsed(EndEffectorParamsNT.CORAL_SHOOT_DELAY_TIME.getValue())) {
                    shootTimer.stop();
                    shootTimer.reset();
                }
            }
        }

    }

    @Override
    public boolean isFinished() {
        return shootFinished;
    }

    @Override
    public void end(boolean interrupted) {
        shootFinished = false;
        elevatorSubsystem.setElevatorPosition(ElevatorCommonNT.INTAKE_EXTENSION_METERS.getValue());
        endEffectorSubsystem.setRollerVoltage(0);
    }

}