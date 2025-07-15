package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;

import static frc.robot.ElevatorCommonNT.INTAKE_EXTENSION_METERS;
import static frc.robot.EndEffectorParamsNT.ALGAE_POKE_VOLTAGE;

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
        endEffectorSubsystem.setRollerVoltage(ALGAE_POKE_VOLTAGE.getValue());
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setElevatorPosition(INTAKE_EXTENSION_METERS.getValue());
        endEffectorSubsystem.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return endEffectorSubsystem.hasCoral();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
