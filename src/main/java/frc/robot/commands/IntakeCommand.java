package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ElevatorCommonNT;
import frc.robot.EndEffectorParamsNT;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;

public class IntakeCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    DestinationSupplier destinationSupplier = DestinationSupplier.getInstance();

    public IntakeCommand(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;
        addRequirements(elevatorSubsystem, endEffectorSubsystem);
    }

    @Override
    public void initialize() {
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
        endEffectorSubsystem.stopRoller();;
    }


}
