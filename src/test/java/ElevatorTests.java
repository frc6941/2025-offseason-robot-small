import javax.swing.text.StyledEditorKit.BoldAction;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import frc.robot.Subsystems.ElevatorSubsystem.ElevatorIO;
import frc.robot.Subsystems.ElevatorSubsystem.ElevatorIOSim;
import frc.robot.Subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.Subsystems.ElevatorSubsystem.ElevatorSubsystem;

public class ElevatorTests {
    private static ElevatorSubsystem elevator;
    @BeforeAll
    static void setup() {
        elevator = new ElevatorSubsystem(new ElevatorIOSim());
    }

    @Test
    void GenerateElevtaor(){
        
        elevator.setGoal(ElevatorState.L1);
        System.out.println(elevator.getGoalPosition());
        elevator.periodic();
        elevator.setGoal(ElevatorState.L2);
        System.out.println(elevator.getGoalPosition());
        elevator.updateSlot0Config();
        elevator.periodic();
        elevator.setGoal(ElevatorState.L3);
        System.out.println(elevator.getGoalPosition());
        elevator.periodic();
        elevator.setGoal(ElevatorState.L4);
        System.out.println(elevator.getGoalPosition());
        elevator.periodic();
        System.out.println(elevator.isZeroing());
        elevator.zeroElevator();
        System.out.println(elevator.isZeroing());
    }

    @AfterEach
    void reset() {
    }
}
