package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.EndEffectorParamsNT;
import frc.robot.Robot;
import frc.robot.RobotStateRecorder;
import frc.robot.display.SuperstructureVisualizer;
import frc.robot.display.SuperstructureVisualizerPose3d;
import frc.robot.drivers.AimGoalSupplier;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import lib.ironpulse.utils.LoggedTracer;
import lib.ironpulse.utils.TimeDelayedBoolean;
import lombok.Builder;
import lombok.Getter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.*;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.SuperstructureState.*;


public class Superstructure extends SubsystemBase {

    private final Graph<SuperstructureState, EdgeCommand> graph = new DefaultDirectedGraph<>(EdgeCommand.class);

    @Getter
    private SuperstructureState state = SuperstructureState.START;
    private SuperstructureState next = null;
    @Getter
    private SuperstructureState goal = SuperstructureState.START;
    private EdgeCommand edgeCommand;
    private final SuperstructureVisualizer measuredPoseVisualizer;
    private final SuperstructureVisualizer setpointPoseVisualizer;
    private final SuperstructureVisualizer goalPoseVisualizer;
    private final SuperstructureVisualizerPose3d measuredPose;
    private final SuperstructureVisualizerPose3d setPointPose;
    private final SuperstructureVisualizerPose3d goalPose;
    private final Set<Pair<SuperstructureState, SuperstructureState>> shootStates;


    private final EndEffectorSubsystem endEffectorSubsystem;
    private final ElevatorSubsystem elevator;
    /**
     * Constructor for the Superstructure subsystem.
     * <p>
     * See README.md for detailed implementation requirements and framework documentation.
     *
     * @see #addEdge(SuperstructureState, SuperstructureState)
     * @see #getEdgeCommand(SuperstructureState, SuperstructureState)
     * @see #setDefaultCommand(Command)
     * @see EdgeCommand
     */
    public Superstructure(EndEffectorSubsystem endEffectorSubsystem, ElevatorSubsystem elevator) {
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.elevator = elevator;
        this.measuredPoseVisualizer = new SuperstructureVisualizer();
        this.setpointPoseVisualizer = new SuperstructureVisualizer();
        this.goalPoseVisualizer = new SuperstructureVisualizer();
        this.goalPose = new SuperstructureVisualizerPose3d();
        this.measuredPose = new SuperstructureVisualizerPose3d();
        this.setPointPose = new SuperstructureVisualizerPose3d();

        // Add states as vertices
        for (var state : SuperstructureState.values()) {
            graph.addVertex(state);
        }

        // Declear all edges here
        addEdge(SuperstructureState.START, SuperstructureState.IDLE, false, false);
        // Add edges between shoot and preshoot states
        this.shootStates =
                Set.of(
                        Pair.of(SuperstructureState.L2, SuperstructureState.L2_EJECT),
                        Pair.of(SuperstructureState.L3, SuperstructureState.L3_EJECT),
                        Pair.of(SuperstructureState.L4, SuperstructureState.L4_EJECT));
        for (var pair : shootStates) {
            addEdge(pair.getFirst(), pair.getSecond(), true, false);
        }

        addEdge(IDLE,INTAKE,true,false);
        addEdge(INTAKE,INDEX,false, false);
        addEdge(IDLE,OUTTAKE,true,true);
        addEdge(INDEX,IDLE,false,false);
        addEdge(IDLE,P1,true,false);
        addEdge(IDLE,P2,true,false);
        addEdge(IDLE,L4,true,false);
        addEdge(IDLE,L3,true,false);
        addEdge(IDLE,L2,true,false);



//        addBidirectionalEdge(RobotSuperstructuresState.L2,RobotSuperstructuresState.L2_EJECT, Commands.none(), Commands.none(), false);
//        addBidirectionalEdge(RobotSuperstructuresState.L3,RobotSuperstructuresState.L3_EJECT, Commands.none(), Commands.none(), false);
//        addBidirectionalEdge(RobotSuperstructuresState.L4,RobotSuperstructuresState.L4_EJECT, Commands.none(), Commands.none(), false);
//
//        addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.INTAKE, null, null, false);
//        addEdge(RobotSuperstructuresState.INTAKE, RobotSuperstructuresState.INDEX, null, false);
//        addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.OUTTAKE, null, null, false);
//        addEdge(RobotSuperstructuresState.INDEX, RobotSuperstructuresState.IDLE, null,  false);
//        addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.L2_EJECT, null, null, false);
//        addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.L3_EJECT, null, null, false);
//        addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.L4_EJECT, null, null, false);
//        addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.P1, null, null, false);
//        addBidirectionalEdge(RobotSuperstructuresState.IDLE, RobotSuperstructuresState.P2, null, null, false);
        setDefaultCommand(
                runGoal(() -> {
                    if ((endEffectorSubsystem.isFrontEE() && !endEffectorSubsystem.isEndEE() )|| (endEffectorSubsystem.isEndEE() && endEffectorSubsystem.isFrontEE())) {
                        return INDEX;
                    }
                    if(endEffectorSubsystem.intakeFinished() && !endEffectorSubsystem.isFrontEE()){
                        return IDLE;
                    }
                    else {
                        return SuperstructureState.IDLE;
                    }

                })
        );
    }

    edu.wpi.first.wpilibj.Timer simTimer = new Timer();

    @Override
    public void periodic() {
        // Set to current on disable
        if(DriverStation.isDisabled()) {
            elevator.setElevatorPosition(elevator::getElevatorPosition);
        }

        // Run periodic
        endEffectorSubsystem.periodic();
        elevator.periodic();

        //simulated gamepiece tracking
        if (Robot.isSimulation()) {
            if(state == START){
                endEffectorSubsystem.setHasCoral(true);
            }
            for (var pair : shootStates) {
                if (state == pair.getSecond()) {
                    // Start the one-shot timer the first time we enter the state
                    if (simTimer.get() == 0) {
                        simTimer.restart();
                    }

                    // Wait 0.5 s before actually clearing the game piece
                    if (simTimer.hasElapsed(0.5)) {
                        endEffectorSubsystem.setFrontEE(false);
                        endEffectorSubsystem.setEndEE(false);
                        endEffectorSubsystem.setHasCoral(false);
                        simTimer.stop();          // reset so it’s ready for the next shot
                        simTimer.reset();
                    }
                }
            }

            if(state == INTAKE){
                endEffectorSubsystem.setHasCoral(true);
                endEffectorSubsystem.setFrontEE(true);
                endEffectorSubsystem.setEndEE(false);
            }
            if(state == INDEX){
                endEffectorSubsystem.setIntakeFinished(true);
                endEffectorSubsystem.setFrontEE(false);
                endEffectorSubsystem.setEndEE(true);
            }
            if (state == P1 || state == P2){
                endEffectorSubsystem.setHasCoral(false);
            }

            //log the gamepiece tracking
            measuredPoseVisualizer.logCoralPose3D(
                endEffectorSubsystem.isHasCoral()
            );

            measuredPoseVisualizer.update(
                elevator.getElevatorPosition()
            );
            setpointPoseVisualizer.update(
                elevator.getWantedPosition()
            );
            goalPoseVisualizer.update(
                goal.getValue().getPose().elevatorHeight().getAsDouble()
            );
        }
        measuredPose.updateVisuals(elevator.getElevatorPosition());
        measuredPose.logCoralPose(endEffectorSubsystem.hasCoral());
        goalPose.updateVisuals(goal.getValue().getPose().elevatorHeight().getAsDouble());
        setPointPose.updateVisuals(elevator.getWantedPosition());
        Logger.recordOutput("Superstructure/endeffectorPose",measuredPose.getEndeffector());
        Logger.recordOutput("Superstructure/elevatorStage1Pose",measuredPose.getElevator1stStage());
        Logger.recordOutput("Superstructure/elevatorStage2Pose",measuredPose.getElevator2ndStage());
        Logger.recordOutput("Superstructure/coralPose",measuredPose.getCoralPose());
        // if we complete the current command, there are three things that we should planning to do
        // 1. update state (to next, which is current state)
        // 2. update next (to the next target state, which should be found through bfs)
        // 3. update edgeCommand (the command we should process to reach next)
        if (edgeCommand == null || !edgeCommand.getCommand().isScheduled()) {
            // this part complete 1.
            if (next != null) {
                // change the old state to current state since we already complete the command
                state = next;
                // change next state to null temporarily
                next = null;
            }

            // now we should complete 2 and 3 by using bfs
            // Schedule next command in sequence
            if (state != goal) {
                // the bfs will find the shortest path between our current state and the goal and
                // return the first point(state) to get to the goal
                bfs(state, goal)
                        .ifPresent(next -> {
                            // this setup the next state
                            this.next = next;
                            // find the edge(command) between state and next
                            edgeCommand = graph.getEdge(state, next);
                            // schedule the command to get to the next state
                            edgeCommand.getCommand().schedule();
                        });
            }
        }

        // Log state
        Logger.recordOutput("Superstructure/State", state);
        if(state != null){
            Logger.recordOutput("Superstructure/StatePose", state.getValue().getPose().elevatorHeight());
        }
        if(goal != null){
            Logger.recordOutput("Superstructure/GoalPose", goal.getValue().getPose().elevatorHeight());
        }
        if(next != null){
        Logger.recordOutput("Superstructure/NextPose", next.getValue().getPose().elevatorHeight());
        }
        Logger.recordOutput("Superstructure/Next", next);
        Logger.recordOutput("Superstructure/Goal", goal);
        if (edgeCommand != null) {
            Logger.recordOutput(
                    "Superstructure/EdgeCommand",
                    graph.getEdgeSource(edgeCommand) + " --> " + graph.getEdgeTarget(edgeCommand));
        } else {
            Logger.recordOutput("Superstructure/EdgeCommand", "");
        }

        // Record cycle time
        LoggedTracer.record("Superstructure");

        SmartDashboard.putString("Superstructure/State", state != null ? state.toString() : "Null");
        SmartDashboard.putString("Superstructure/Next", next != null ? next.toString() : "Null");
        SmartDashboard.putString("Superstructure/Goal", goal != null ? goal.toString() : "Null");
        SmartDashboard.putBoolean("Superstructure/PoseAtGoal", poseAtGoal());
    }

    @AutoLogOutput(key = "Superstructure/AtGoal")
    public boolean atGoal() {
        return state == goal;
    }


    public Command runGoal(SuperstructureState goal) {
        return runOnce(() -> setGoal(goal)).andThen(Commands.idle(this));
    }

    public Command runGoal(Supplier<SuperstructureState> goal) {
        return run(() -> setGoal(goal.get()));
    }

    public Command runZero() {
        return elevator.zeroElevator().andThen(Commands.print("Zero Success!"));
    }

    public void startNormal() {
        state = START;
    }

    public void startAuto() {
        state = START;
    }


    public void setGoal(SuperstructureState goal) {
        // Don't do anything if goal is the same
        if (this.goal == goal) return;
        this.goal = goal;

        if (next == null) return;

        var edgeToCurrentState = graph.getEdge(next, state);
        // Figure out if we should schedule a different command to get to goal faster
        if (edgeCommand.getCommand().isScheduled()
                && edgeToCurrentState != null
                && isEdgeAllowed(edgeToCurrentState, goal)) {
            // Figure out where we would have gone from the previous state
            bfs(state, goal)
                    .ifPresent(newNext -> {
                        if (newNext == next) {
                            // We are already on track
                            return;
                        }

                        if (newNext != state && graph.getEdge(next, newNext) != null) {
                            // We can skip directly to the newNext edge
                            edgeCommand.getCommand().cancel();
                            edgeCommand = graph.getEdge(state, newNext);
                            edgeCommand.getCommand().schedule();
                            next = newNext;
                        } else {
                            // Follow the reverse edge from next back to the current edge
                            edgeCommand.getCommand().cancel();
                            edgeCommand = graph.getEdge(next, state);
                            edgeCommand.getCommand().schedule();
                            var temp = state;
                            state = next;
                            next = temp;
                        }
                    });
        }
    }

    private Optional<SuperstructureState> bfs(SuperstructureState start, SuperstructureState goal) {
        // Map to track the parent of each visited node
        Map<SuperstructureState, SuperstructureState> parents = new HashMap<>();
        Queue<SuperstructureState> queue = new LinkedList<>();
        queue.add(start);
        parents.put(start, null); // Mark the start node as visited with no parent

        // Perform BFS
        while (!queue.isEmpty()) {
            SuperstructureState current = queue.poll();
            // Check if we've reached the goal
            if (current == goal) {
                break;
            }
            // Process valid neighbors
            for (EdgeCommand edge : graph.outgoingEdgesOf(current)) {
                SuperstructureState neighbor = graph.getEdgeTarget(edge);
                // Only process unvisited neighbors
                if (!parents.containsKey(neighbor)) {
                    parents.put(neighbor, current);
                    queue.add(neighbor);
                }
            }
        }

        // Reconstruct the path to the goal if found
        if (!parents.containsKey(goal)) {
            return Optional.empty(); // Goal not reachable
        }

        // Trace back the path from goal to start
        SuperstructureState nextState = goal;
        while (!nextState.equals(start)) {
            SuperstructureState parent = parents.get(nextState);
            if (parent == null) {
                return Optional.empty(); // No valid path found
            } else if (parent.equals(start)) {
                // Return the edge from start to the next node
                return Optional.of(nextState);
            }
            nextState = parent;
        }
        return Optional.of(nextState);
    }

    private boolean isEdgeAllowed(EdgeCommand edge, SuperstructureState goal) {
        return !edge.isRestricted() || goal == graph.getEdgeTarget(edge);
    }

    /**
     * Adds an edge between two states in the superstructure state machine.
     *
     * @param from       The source state
     * @param to         The target state
     * @param reverse    If true, also adds a reverse edge from 'to' to 'from'
     * @param restricted If true, this edge can only be used when transitioning directly to its target state(z.b from L4 to L4shoot))
     * @see #isEdgeAllowed(EdgeCommand, SuperstructureState) Implementation of restricted edge behavior
     * @see #bfs(SuperstructureState, SuperstructureState) Path finding that respects restricted edges
     */
    private void addEdge(
            SuperstructureState from,
            SuperstructureState to,
            boolean reverse,
            boolean restricted) {
        graph.addEdge(
                from,
                to,
                EdgeCommand.builder()
                        .command(getEdgeCommand(from, to))
                        .restricted(restricted)
                        .build());
        if (reverse) {
            graph.addEdge(
                    to,
                    from,
                    EdgeCommand.builder()
                            .command(getEdgeCommand(to, from))
                            .restricted(restricted)
                            .build());
        }
    }

    /**
     * Adds a non-reversible edge between two states.
     *
     * @param from       The source state
     * @param to         The target state
     * @param restricted If true, this edge can only be used when transitioning directly to its target state
     */
    private void addEdge(SuperstructureState from, SuperstructureState to, boolean restricted) {
        addEdge(from, to, false, restricted);
    }

    /**
     * Adds a non-reversible, non-restricted edge between two states.
     *
     * @param from The source state
     * @param to   The target state
     */
    private void addEdge(SuperstructureState from, SuperstructureState to) {
        addEdge(from, to, false);
    }


    /**
     * All edge commands should finish and exit properly.
     */
    @Builder(toBuilder = true)
    @Getter
    public static class EdgeCommand extends DefaultEdge {
        private final Command command;
        @Builder.Default
        private final boolean restricted = false;
    }

    public boolean hasCoral() {
        return endEffectorSubsystem.hasCoral();
    }

    public double getElevatorPosition() {
        return elevator.getElevatorPosition();
    }
    private Command runElevator(DoubleSupplier position) {
        return Commands.runOnce(() -> elevator.setElevatorPosition(position));
    }

    /**
     * Runs elevator and pivot to {@link SuperstructurePose} pose. Ends immediately.
     */
    private Command runSuperstructurePose(SuperstructurePose pose) {
        return runElevator(pose.elevatorHeight());
    }

    public boolean poseAtGoal() {
        return elevator.isAtGoal() /*&& intake.isAtGoal()*/;
    }

    private Command runSuperstructureRollers(SuperstructureState state) {
        return Commands.runOnce(() -> {
            endEffectorSubsystem.setRollerVoltage(state.getValue().getEndEffectorVolts());
        });
    }

    // declare all edge commands here
    private Command getEdgeCommand(SuperstructureState from, SuperstructureState to) {
        if (from == SuperstructureState.START && to == SuperstructureState.IDLE) {
            return Commands.sequence(
                    elevator.zeroElevator(),
                    Commands.waitUntil(() -> !elevator.isZeroing())
            );
        }
        else {
            return runSuperstructurePose(to.getValue().getPose())
                    .until(elevator::isAtGoal)
                    .andThen(runSuperstructureRollers(to));
        }

    }

    public static SuperstructureState elevatorPoking(ElevatorSetPoint setPoint){
        return switch (setPoint){
            case P1 -> P1;
            case P2 -> P2;
            default -> IDLE;
        };
    }

    public static SuperstructureState elevatorNormal(ElevatorSetPoint setPoint){
        return switch (setPoint){
            case L2 -> L2;
            case L3 -> L3;
            case L4 -> L4;

            default -> IDLE;        };
    }

    public static SuperstructureState elevatorforShoot(ElevatorSetPoint setPoint){
        return switch (setPoint){
            case L2 -> L2_EJECT;
            case L3 -> L3_EJECT;
            case L4 -> L4_EJECT;
            default -> IDLE;
        };
    }

    public Command shoot(){
        return new ConditionalCommand(endEffectorSubsystem.setRolelrVoltage(EndEffectorParamsNT.CORAL_SHOOT_VOLTAGE::getValue).until(()->!endEffectorSubsystem.hasCoral()),
                endEffectorSubsystem.setRolelrVoltage(EndEffectorParamsNT.CORAL_SHOOT_VOLTAGE::getValue).alongWith(Commands.runOnce(()->endEffectorSubsystem.setHasCoral(false))).andThen(new WaitCommand(0.5)).until(()->!endEffectorSubsystem.hasCoral()),
                ()->!Robot.isSimulation());
    }



}