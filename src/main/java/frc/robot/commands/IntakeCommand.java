package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ElevatorCommonNT;
import frc.robot.EndEffectorParamsNT;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO.Patterns;

public class IntakeCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    DestinationSupplier destinationSupplier = DestinationSupplier.getInstance();

    public IntakeCommand(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem, IndicatorSubsystem indicatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        addRequirements(elevatorSubsystem, endEffectorSubsystem);
    }

    @Override
    public void initialize() {
        indicatorSubsystem.setPattern(Patterns.INTAKE);
        endEffectorSubsystem.setRollerVoltage(EndEffectorParamsNT.CORAL_INTAKE_VOLTAGE.getValue()); 
        destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.INTAKE);  
        elevatorSubsystem.setElevatorPosition(destinationSupplier.getElevatorSetpoint(true));
    }

    @Override
    public void execute() {
        if(endEffectorSubsystem.hasCoral()){
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
        indicatorSubsystem.setPattern(Patterns.NORMAL);
        destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L2);  
        elevatorSubsystem.setElevatorPosition(destinationSupplier.getElevatorSetpoint(true));
    }


}
