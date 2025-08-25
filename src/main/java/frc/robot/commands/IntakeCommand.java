package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.ElevatorCommonNT;
import frc.robot.EndEffectorParamsNT;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO.Patterns;
import frc.robot.subsystems.indicator.IndicatorSubsystem;

import static edu.wpi.first.units.Units.Seconds;

@Deprecated
public class IntakeCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final CommandXboxController controller = new CommandXboxController(0);
    DestinationSupplier destinationSupplier = DestinationSupplier.getInstance();
    private final boolean enableRumble;

    public IntakeCommand(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, IndicatorSubsystem indicatorSubsystem) {
        this(elevatorSubsystem, endEffectorSubsystem, indicatorSubsystem, false);
    }

    public IntakeCommand(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, IndicatorSubsystem indicatorSubsystem, boolean enableRumble) {
        this.enableRumble = enableRumble;
        this.elevatorSubsystem = elevatorSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        addRequirements(elevatorSubsystem, endEffectorSubsystem);
    }


    @Override
    public void initialize() {
        indicatorSubsystem.setPattern(Patterns.INTAKE);
        endEffectorSubsystem.setRollerVoltage(EndEffectorParamsNT.CORAL_INTAKE_VOLTAGE.getValue());
        elevatorSubsystem.setElevatorPosition(ElevatorCommonNT.INTAKE_EXTENSION_METERS.getValue());
    }

    @Override
    public void execute() {
        if (endEffectorSubsystem.hasCoral()) {
            endEffectorSubsystem.setRollerVoltage(EndEffectorParamsNT.CORAL_INDEX_VOLTAGE.getValue());
        }
    }

    @Override
    public boolean isFinished() {
        return endEffectorSubsystem.intakeFinished();
    }

    @Override
    public void end(boolean interrupted) {
        endEffectorSubsystem.stopRoller();
        if (!interrupted) {
            indicatorSubsystem.setPattern(Patterns.AFTER_INTAKE);
            if (enableRumble)
                CommandScheduler.getInstance().schedule(new RumbleCommand(Seconds.of(1), controller.getHID()));
        } else indicatorSubsystem.setNormal();
        elevatorSubsystem.setElevatorPosition(ElevatorCommonNT.INTAKE_EXTENSION_METERS.getValue());
    }
}
