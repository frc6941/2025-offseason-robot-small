package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ElevatorCommonNT;
import frc.robot.EndEffectorParamsNT;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;

@Deprecated
public class PokeCommand extends Command {
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    public PokeCommand(EndEffectorSubsystem endEffectorSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(endEffectorSubsystem, elevatorSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setElevatorPosition(DestinationSupplier.getInstance().getElevatorSetpoint(false));
        endEffectorSubsystem.setRollerVoltage(EndEffectorParamsNT.ALGAE_POKE_VOLTAGE.getValue());
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setElevatorPosition(ElevatorCommonNT.INTAKE_EXTENSION_METERS.getValue());
        endEffectorSubsystem.setRollerVoltage(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}