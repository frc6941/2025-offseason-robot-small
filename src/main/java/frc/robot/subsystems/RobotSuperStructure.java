package frc.robot.subsystems;

import static frc.robot.Ports.END_EFFECTOR;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ElevatorCommonNT;
import frc.robot.EndEffectorParamsNT;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import lib.ironpulse.superstructure.*;

public class RobotSuperStructure extends SuperstructureLibrary<frc.robot.subsystems.RobotSuperStructure.RobotSuperstructuresState>{
    private final ElevatorSubsystem elevatorSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;

    SuperstructurePose idlePose;
    SuperstructurePose intakePose;
    SuperstructurePose indexPose;
    SuperstructurePose outtakePose;
    SuperstructurePose L2;
    SuperstructurePose L3;
    SuperstructurePose L4;
    SuperstructurePose P1;
    SuperstructurePose P2;

    public RobotSuperStructure(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
        super();
        this.elevatorSubsystem = elevatorSubsystem;
        this.endEffectorSubsystem = endEffectorSubsystem;

        Init();

        setDefaultCommand(runGoal(() -> RobotSuperstructuresState.IDLE));
        
    }

    @Override
    public void periodic() {
        if(DriverStation.isDisabled()) {
            elevatorSubsystem.setElevatorPosition(elevatorSubsystem::getElevatorPosition);
        }

        elevatorSubsystem.periodic();
    }



    private void Init(){
        // Define poses
        setPose();

        //set states
        setState();

        // Set edges
        setEdge();
 
    }

    public void setPose(){
        idlePose = SuperstructurePoseBuilder.create("IDLE")
                .withParameter("elevator", ElevatorCommonNT.INTAKE_EXTENSION_METERS::getValue)
                .build();
        intakePose = SuperstructurePoseBuilder.create("INTAKE")
                .withParameter("elevator", ElevatorCommonNT.INTAKE_EXTENSION_METERS::getValue)
                .build();
        indexPose = SuperstructurePoseBuilder.create("INDEX")
                .withParameter("elevator", ElevatorCommonNT.INTAKE_EXTENSION_METERS::getValue)
                .build();
        outtakePose = SuperstructurePoseBuilder.create("OUTTAKE")
                .withParameter("elevator", ElevatorCommonNT.INTAKE_EXTENSION_METERS::getValue)
                .build();
        L2 = SuperstructurePoseBuilder.create("L2")
                .withParameter("elevator", ElevatorCommonNT.L2_EXTENSION_METERS::getValue)
                .build();
        L3 = SuperstructurePoseBuilder.create("L3")
                .withParameter("elevator", ElevatorCommonNT.L3_EXTENSION_METERS::getValue)
                .build();
        L4 = SuperstructurePoseBuilder.create("L4")
                .withParameter("elevator", ElevatorCommonNT.L4_EXTENSION_METERS::getValue)
                .build();
        P1 = SuperstructurePoseBuilder.create("P1")
                .withParameter("elevator", ElevatorCommonNT.P1_EXTENSION_METERS::getValue)
                .build();
        P2 = SuperstructurePoseBuilder.create("P2")
                .withParameter("elevator", ElevatorCommonNT.P2_EXTENSION_METERS::getValue)
                .build();
    }

    private void setState(){
        addState(RobotSuperstructuresState.IDLE,
                SuperstructureStateBuilder.create(
                        "IDLE", idlePose)
                        .withParameter("endEffectorVolts", EndEffectorParamsNT.IDLE_VOLTAGE::getValue)
                        .build());

        addState(RobotSuperstructuresState.INTAKE,
                SuperstructureStateBuilder.create(
                                "INTAKE", intakePose)
                        .withParameter("endEffectorVolts", EndEffectorParamsNT.CORAL_INTAKE_VOLTAGE::getValue)
                        .build());

        addState(RobotSuperstructuresState.INDEX,
                SuperstructureStateBuilder.create(
                        "INDEX", intakePose)
                        .withParameter("endEffectorVolts", EndEffectorParamsNT.CORAL_INDEX_VOLTAGE::getValue)
                        .build());

        addState(RobotSuperstructuresState.OUTTAKE,
                SuperstructureStateBuilder.create(
                                "OUTTAKE", outtakePose)
                        .withParameter("endEffectorVolts", EndEffectorParamsNT.CORAL_SHOOT_VOLTAGE::getValue)
                        .build());

        addState(RobotSuperstructuresState.L2,
                SuperstructureStateBuilder.create(
                                "L2", L2)
                        .withParameter("endEffectorVolts", EndEffectorParamsNT.IDLE_VOLTAGE::getValue)
                        .build());

        addState(RobotSuperstructuresState.L2_EJECT,
                SuperstructureStateBuilder.create(
                                "L2_EJECT", L2)
                        .withParameter("endEffectorVolts", EndEffectorParamsNT.CORAL_SHOOT_VOLTAGE::getValue)
                        .build());
        addState(RobotSuperstructuresState.L3,
                SuperstructureStateBuilder.create(
                                "L3", L3)
                        .withParameter("endEffectorVolts", EndEffectorParamsNT.IDLE_VOLTAGE::getValue)
                        .build());
        addState(RobotSuperstructuresState.L3_EJECT,
                SuperstructureStateBuilder.create(
                                "L3_EJECT", L3)
                        .withParameter("endEffectorVolts", EndEffectorParamsNT.CORAL_SHOOT_VOLTAGE::getValue)
                        .build());
        addState(RobotSuperstructuresState.L4,
                SuperstructureStateBuilder.create(
                                "L4", L4)
                        .withParameter("endEffectorVolts", EndEffectorParamsNT.IDLE_VOLTAGE::getValue)
                        .build());
        addState(RobotSuperstructuresState.L4_EJECT,
                SuperstructureStateBuilder.create(
                                "L4_EJECT", L4)
                        .withParameter("endEffectorVolts", EndEffectorParamsNT.CORAL_SHOOT_VOLTAGE::getValue)
                        .build());
        addState(RobotSuperstructuresState.P1,
                SuperstructureStateBuilder.create(
                                "P1", P1)
                        .withParameter("endEffectorVolts", EndEffectorParamsNT.ALGAE_POKE_VOLTAGE::getValue)
                        .build());
        addState(RobotSuperstructuresState.P2,
                SuperstructureStateBuilder.create(
                                "P2", P2)
                        .withParameter("endEffectorVolts", EndEffectorParamsNT.ALGAE_POKE_VOLTAGE::getValue)
                        .build());
        }

        public void setEdge(){
                addBidirectionalEdge(RobotSuperstructuresState.L2,RobotSuperstructuresState.L2_EJECT, Commands.none(), Commands.none(), false);
                addBidirectionalEdge(RobotSuperstructuresState.L3,RobotSuperstructuresState.L3_EJECT, Commands.none(), Commands.none(), false);
                addBidirectionalEdge(RobotSuperstructuresState.L4,RobotSuperstructuresState.L4_EJECT, Commands.none(), Commands.none(), false);

                addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.INTAKE, null, null, false);
                addEdge(RobotSuperstructuresState.INTAKE, RobotSuperstructuresState.INDEX, null, false);
                addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.OUTTAKE, null, null, false);
                addEdge(RobotSuperstructuresState.INDEX, RobotSuperstructuresState.IDLE, null,  false);
                addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.L2_EJECT, null, null, false);
                addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.L3_EJECT, null, null, false);
                addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.L4_EJECT, null, null, false);
                addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.P1, null, null, false);
                addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.P2, null, null, false);
        }
    

        private Command runElevator(DoubleSupplier position) {
                return Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(position));
        }


        public enum RobotSuperstructuresState implements SuperstructureState{
                IDLE,
                INTAKE,
                INDEX,
                OUTTAKE,
                L2,
                L2_EJECT,
                L3,
                L3_EJECT,
                L4,
                L4_EJECT,
                P1,
                P2
        }
}
    


