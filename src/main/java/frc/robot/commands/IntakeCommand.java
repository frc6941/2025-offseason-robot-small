package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;

public class IntakeCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;

    public IntakeCommand(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        addRequirements(elevatorSubsystem, endEffectorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return endEffectorSubsystem.intakeFinished();
    }

    @Override
    public void end(boolean interrupted) {
    }


}
