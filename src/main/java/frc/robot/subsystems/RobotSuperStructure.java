package frc.robot.subsystems;

import static frc.robot.Ports.END_EFFECTOR;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ElevatorCommonNT;
import frc.robot.EndEffectorParamsNT;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import lib.ironpulse.superstructure.*;
import lib.ironpulse.utils.LoggedTracer;

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

        setCurrentState(RobotSuperstructuresState.IDLE);
        setNextState(RobotSuperstructuresState.IDLE);
        setGoal(RobotSuperstructuresState.IDLE);
        
    }

    @Override
    public void periodic() {
        super.periodic();
        if(DriverStation.isDisabled()) {
            elevatorSubsystem.setElevatorPosition(elevatorSubsystem::getElevatorPosition);
        }

        elevatorSubsystem.periodic();

        Logger.recordOutput("Superstructure/State", getCurrentState());
        Logger.recordOutput("Superstructure/Next", getNextState());
        Logger.recordOutput("Superstructure/Goal", getGoalState());
        Logger.recordOutput("Superstructure/isAtGoal", atGoal());

        if (getCurrentEdge() != null) {
                Logger.recordOutput(
                        "Superstructure/EdgeCommand",
                        graph.getEdgeSource(getCurrentEdge()) + " --> " + graph.getEdgeTarget(getCurrentEdge()));
            } else {
                Logger.recordOutput("Superstructure/EdgeCommand", "");
            }
        LoggedTracer.record("Superstructure");

        SmartDashboard.putString("Superstructure/State", getCurrentState() != null ? getCurrentState().toString() : "Null");
        SmartDashboard.putString("Superstructure/Next", getNextState() != null ? getNextState().toString() : "Null");
        SmartDashboard.putString("Superstructure/Goal", getGoalState() != null ? getGoalState().toString() : "Null");
        SmartDashboard.putBoolean("Superstructure/PoseAtGoal", poseAtGoal());
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

        private void setEdge(){
                addEdge(RobotSuperstructuresState.L2,RobotSuperstructuresState.L2_EJECT, JustShootCommand(), false);
                addEdge(RobotSuperstructuresState.L3,RobotSuperstructuresState.L3_EJECT, JustShootCommand(), false);
                addEdge(RobotSuperstructuresState.L4,RobotSuperstructuresState.L4_EJECT, JustShootCommand(), false);

                addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.INTAKE, intakeCommand(), idleCommand(), false);
                addEdge(RobotSuperstructuresState.INTAKE, RobotSuperstructuresState.INDEX, indexCommand(), false);
                addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.OUTTAKE, outtakeCommand(), idleCommand(), false);
                addEdge(RobotSuperstructuresState.INDEX, RobotSuperstructuresState.IDLE, idleCommand(),  false);
                addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.L2_EJECT, scoreCommand(elevatorSetpoint.L2), idleCommand(), false);
                addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.L3_EJECT, scoreCommand(elevatorSetpoint.L3), idleCommand(), false); 
                addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.L4_EJECT, scoreCommand(elevatorSetpoint.L4), idleCommand(), false);
                addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.L2, ElevatorGotoScorePos(elevatorSetpoint.L2), idleCommand(), false);
                addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.L3, ElevatorGotoScorePos(elevatorSetpoint.L3), idleCommand(), false); 
                addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.L4, ElevatorGotoScorePos(elevatorSetpoint.L4), idleCommand(), false);
                addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.P1, SuperstructurePokeCommand(elevatorSetpoint.P1), idleCommand(), false);
                addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.P2, SuperstructurePokeCommand(elevatorSetpoint.P2), idleCommand(), false);
        }

        private Command JustShootCommand(){
                return runEndEffector(EndEffectorParamsNT.CORAL_SHOOT_VOLTAGE::getValue).until(()->!endEffectorSubsystem.hasCoral());
        }

        private Command idleCommand(){
                return runEndEffector(EndEffectorParamsNT.IDLE_VOLTAGE::getValue).alongWith(runElevator(()->idlePose.getParameter("elevator")));
        }

        private Command ElevatorGotoScorePos(elevatorSetpoint setpoint){
                if(setpoint == elevatorSetpoint.L2){
                        return runElevator(()->L2.getParameter("elevator"));
                }else if(setpoint == elevatorSetpoint.L3){
                        return runElevator(()->L3.getParameter("elevator"));
                }else if(setpoint == elevatorSetpoint.L4){
                        return runElevator(()->L4.getParameter("elevator"));
                }else{
                        return Commands.print("Illegal Elevator Setpoint");
                }
        }

        private Command scoreCommand(elevatorSetpoint setpoint){
                if(setpoint == elevatorSetpoint.L2){
                        return ElevatorGotoScorePos(elevatorSetpoint.L2).until(()->elevatorSubsystem.isAtGoal()).andThen(JustShootCommand()).finallyDo(()->endEffectorSubsystem.stopRoller());
                }else if(setpoint == elevatorSetpoint.L3){
                        return ElevatorGotoScorePos(elevatorSetpoint.L3).until(()->elevatorSubsystem.isAtGoal()).andThen(JustShootCommand()).finallyDo(()->endEffectorSubsystem.stopRoller());
                }else if(setpoint == elevatorSetpoint.L4){
                        return ElevatorGotoScorePos(elevatorSetpoint.L4).until(()->elevatorSubsystem.isAtGoal()).andThen(JustShootCommand()).finallyDo(()->endEffectorSubsystem.stopRoller());
                }else{
                        return Commands.print("Illegal Param for Score");
                }
        }

        private Command SuperstructurePokeCommand(elevatorSetpoint setpoint){
                if(setpoint == elevatorSetpoint.P1){
                        return runElevator(()->P1.getParameter("elevator")).until(()->elevatorSubsystem.isAtGoal()).andThen(runEndEffector(EndEffectorParamsNT.ALGAE_POKE_VOLTAGE::getValue)).finallyDo(()->endEffectorSubsystem.stopRoller());
                }else if(setpoint == elevatorSetpoint.P2){
                        return runElevator(()->P2.getParameter("elevator")).until(()->elevatorSubsystem.isAtGoal()).andThen(runEndEffector(EndEffectorParamsNT.ALGAE_POKE_VOLTAGE::getValue)).finallyDo(()->endEffectorSubsystem.stopRoller());
                }else{
                        return Commands.print("Illegal Param for Poke");
                }
        }

        private Command outtakeCommand(){
                return runEndEffector(EndEffectorParamsNT.CORAL_SHOOT_VOLTAGE::getValue).alongWith(runElevator(()->outtakePose.getParameter("elevator")));
        }

        private Command intakeCommand(){
                return runEndEffector(EndEffectorParamsNT.CORAL_INTAKE_VOLTAGE::getValue).alongWith(runElevator(()->intakePose.getParameter("elevator"))).until(()->endEffectorSubsystem.hasCoral()).andThen(runGoal(RobotSuperstructuresState.INDEX));
        }

        private Command indexCommand(){
                return runEndEffector(EndEffectorParamsNT.CORAL_INDEX_VOLTAGE::getValue).alongWith(runElevator(()->indexPose.getParameter("elevator"))).until(()->endEffectorSubsystem.intakeFinished()).andThen(runGoal(RobotSuperstructuresState.IDLE));
        }

    
        private Command runElevator(DoubleSupplier position) {
                return Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(position));
        }

        private Command runEndEffector(DoubleSupplier volts) {
                return Commands.runOnce(() -> endEffectorSubsystem.setRollerVoltage(volts));
        }

        public boolean poseAtGoal(){
                return elevatorSubsystem.isAtGoal();
        }

        public RobotSuperstructuresState checkShootingState(){
            return switch (getCurrentState()) {
                case L2 -> RobotSuperstructuresState.L2_EJECT;
                case L3 -> RobotSuperstructuresState.L3_EJECT;
                case L4 -> RobotSuperstructuresState.L4_EJECT;
                default -> getCurrentState();
            };
        }

        
        public static RobotSuperstructuresState elevatorNormalSetpointMapping(elevatorSetpoint setpoint) {
            return switch (setpoint) {
                case L2 -> RobotSuperstructuresState.L2;
                case L3 -> RobotSuperstructuresState.L3;
                case L4 -> RobotSuperstructuresState.L4;
                case P1 -> RobotSuperstructuresState.P1;
                case P2 -> RobotSuperstructuresState.P2;
                default -> RobotSuperstructuresState.IDLE;
            };
            }
            public static RobotSuperstructuresState elevatorScoringSetpointMapping(elevatorSetpoint setpoint) {
                return switch (setpoint) {
                    case L2 -> RobotSuperstructuresState.L2_EJECT;
                    case L3 -> RobotSuperstructuresState.L3_EJECT;
                    case L4 -> RobotSuperstructuresState.L4_EJECT;
                    default -> RobotSuperstructuresState.IDLE;
                };
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

        public enum elevatorSetpoint {
                IDLE, L2, L3, L4, P1, P2
        }
}
    


