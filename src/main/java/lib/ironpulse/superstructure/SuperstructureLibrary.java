package lib.ironpulse.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultEdge;

import java.util.*;
import java.util.function.Supplier;

/**
 * Generic superstructure library for coordinating multiple mechanisms.
 * Provides state machine functionality for any set of mechanisms that need coordinated control.
 * 
 * Usage:
 * 1. Create an enum implementing SuperstructureState
 * 2. Register mechanisms with registerMechanism()
 * 3. Add states with addState()
 * 4. Add edges between states with addEdge()
 * 5. Use runGoal() to transition between states
 */
public class SuperstructureLibrary<T extends SuperstructureState> extends SubsystemBase {
    public final Graph<T, EdgeCommand> graph = new DefaultDirectedGraph<>(EdgeCommand.class);
    
    private T currentState;
    private T nextState;
    private T goalState;
    private EdgeCommand currentEdge;
    
    private final Map<T, SuperstructureStateData> stateDataMap = new HashMap<>();
    
    /**
     * Creates a new SuperstructureLibrary instance.
     */
    public SuperstructureLibrary() {
        this.currentState = null;
        this.nextState = null;
        this.goalState = null;
    }
    
    
    /**
     * Adds a state to the superstructure.
     * @param state The state identifier
     * @param data The state data including pose and actuator commands
     */
    public void addState(T state, SuperstructureStateData data) {
        stateDataMap.put(state, data);
        graph.addVertex(state);
    }
    
    /**
     * Adds an edge between two states.
     * @param from The source state
     * @param to The target state
     * @param transitionCommand The command to execute for this transition
     * @param isRestricted Whether this edge is restricted (only usable for direct transitions)
     */
    public void addEdge(T from, T to, Command transitionCommand, boolean isRestricted) {
        if (!stateDataMap.containsKey(from) || !stateDataMap.containsKey(to)) {
            throw new IllegalArgumentException("Both states must be registered before adding edge");
        }
        
        graph.addEdge(from, to, new EdgeCommand(transitionCommand, isRestricted));
    }
    
    /**
     * Adds a bidirectional edge between two states.
     * @param from The first state
     * @param to The second state
     * @param forwardCommand Command for from->to transition
     * @param reverseCommand Command for to->from transition
     * @param isRestricted Whether these edges are restricted
     */
    public void addBidirectionalEdge(T from, T to, 
                                   Command forwardCommand, Command reverseCommand, boolean isRestricted) {
        addEdge(from, to, forwardCommand, isRestricted);
        addEdge(to, from, reverseCommand, isRestricted);
    }
    
    /**
     * Adds edges between all states in the provided collection.
     * @param states Collection of states to connect
     * @param transitionGenerator Function to generate transition commands between states
     */
    public void addAllToAll(Collection<T> states, 
                          java.util.function.BiFunction<T, T, Command> transitionGenerator) {
        for (T from : states) {
            for (T to : states) {
                if (from != to) {
                    addEdge(from, to, transitionGenerator.apply(from, to), false);
                }
            }
        }
    }
    
    /**
     * Sets the goal state and initiates movement if not already at goal.
     * @param goal The desired goal state
     */
    public void setGoal(T goal) {
        if (goalState == goal) return;
        
        this.goalState = goal;
        
        if (currentEdge == null || !currentEdge.command.isScheduled()) {
            planNextMove();
        }
    }
    
    /**
     * Creates a command to set the goal state.
     * @param goal The goal state
     * @return A command that sets and maintains the goal
     */
    public Command runGoal(T goal) {
        return Commands.runOnce(() -> setGoal(goal)).andThen(Commands.waitUntil(this::atGoal));
    }

    public void setCurrentState(T state){
        currentState = state;
    }

    public void setNextState(T state){
        nextState = state;
    }
    
    /**
     * Creates a command with dynamic goal selection.
     * @param goalSupplier Supplier for the goal state
     * @return A command that continuously updates the goal
     */
    public Command runGoal(Supplier<T> goalSupplier) {
        return Commands.run(() -> setGoal(goalSupplier.get()));
    }
    
    private void planNextMove() {
        if (currentState == null || goalState == null || currentState == goalState) {
            return;
        }
        
        Optional<T> next = breadthFirstSearch(currentState, goalState);
        next.ifPresent(state -> {
            nextState = state;
            currentEdge = graph.getEdge(currentState, state);
            if (currentEdge != null) {
                currentEdge.command.schedule();
            }
        });
    }
    
    private Optional<T> breadthFirstSearch(T start, T goal) {
        Map<T, T> parents = new HashMap<>();
        Queue<T> queue = new LinkedList<>();
        
        queue.add(start);
        parents.put(start, null);
        
        while (!queue.isEmpty()) {
            T current = queue.poll();
            
            if (current == goal) {
                break;
            }
            
            for (EdgeCommand edge : graph.outgoingEdgesOf(current)) {
                @SuppressWarnings("unchecked")
                T neighbor = (T) graph.getEdgeTarget(edge);
                if (!parents.containsKey(neighbor) && (!edge.isRestricted || goal == neighbor)) {
                    parents.put(neighbor, current);
                    queue.add(neighbor);
                }
            }
        }
        
        if (!parents.containsKey(goal)) {
            return Optional.empty();
        }
        
        T nextState = goal;
        while (parents.get(nextState) != null && !parents.get(nextState).equals(start)) {
            nextState = parents.get(nextState);
        }
        
        return Optional.of(nextState);
    }
    
    /**
     * Checks if the superstructure is at the goal state.
     * @return true if at goal
     */
    public boolean atGoal() {
        return currentState == goalState;
    }

    public EdgeCommand getCurrentEdge() {
        return currentEdge;
    }
    
    /**
     * Gets the current state.
     * @return The current state
     */
    public T getCurrentState() {
        return currentState;
    }

    /**
     * Gets the mext state.
     * @return The next state
     */
    public T getNextState() {
        return nextState;
    }
    
    /**
     * Gets the goal state.
     * @return The goal state
     */
    public T getGoalState() {
        return goalState;
    }
    
    @Override
    public void periodic() {
        
        // Check for completed transitions
        if (currentEdge != null && !currentEdge.command.isScheduled()) {
            if (nextState != null) {
                currentState = nextState;
                nextState = null;
            }
            
            if (currentState != goalState) {
                planNextMove();
            }
        }
    }
    
    /**
     * Interface for mechanisms controlled by the superstructure.
     */
    
    /**
     * Edge command wrapper for graph edges.
     */
    private static class EdgeCommand extends DefaultEdge {
        private final Command command;
        private final boolean isRestricted;
        
        EdgeCommand(Command command, boolean isRestricted) {
            this.command = command;
            this.isRestricted = isRestricted;
        }
    }
}