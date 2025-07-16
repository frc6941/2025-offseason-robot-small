package lib.ironpulse.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import lombok.Getter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;

import java.util.function.BooleanSupplier;

public class DecisionTree {
  // fields
  private final Graph<Command, BooleanSupplier> graph;
  private Command rootCommand;
  @Getter
  private Command currentCommand;
  private boolean shouldAdvanceCurrent = false;

  public DecisionTree(Command rootCommand) {
    this.graph = new DefaultDirectedGraph<>(BooleanSupplier.class);
    this.rootCommand = rootCommand;

    registerCallback();
  }

  public DecisionTree() {
    this.graph = new DefaultDirectedGraph<>(BooleanSupplier.class);
    registerCallback();
  }

  public void registerCallback() {
    CommandScheduler.getInstance().onCommandFinish(command -> {
      if(command == currentCommand) shouldAdvanceCurrent = true;
    });
  }

  public DecisionTree addRoot(Command root) {
    graph.addVertex(root);
    rootCommand = root;
    return this;
  }

  public DecisionTree addDecision(
      Command from, Command to,
      BooleanSupplier condition) {
    graph.addVertex(from);
    graph.addVertex(to);
    graph.addEdge(from, to, condition);
    return this;
  }

  public DecisionTree addAlwaysTrueDecision(Command from, Command to) {
    BooleanSupplier alwaysTrue = new BooleanSupplier() {
      @Override
      public boolean getAsBoolean() {
        return true;
      }
    };

    graph.addVertex(from);
    graph.addVertex(to);
    graph.addEdge(from, to, alwaysTrue); // NOTE: need to make a new BooleanSupplier each time
    return this;
  }

  public void reset() {
    currentCommand = null;
  }

  /**
   * Call this every scheduler cycle. It will:
   * 1) schedule root on first tick
   * 2) when currentCommand finishes, test outgoing edges in iteration order
   * and schedule the first target whose supplier returns true.
   */
  public void tick() {
    // 1. start the root if nothing is running yet
    if (currentCommand == null) {
      if (rootCommand != null) {
        rootCommand.schedule();
        currentCommand = rootCommand;
      }
      return;
    }

    // 2. if currentCommand is done, try transitions
    if (shouldAdvanceCurrent) {
      shouldAdvanceCurrent = false;
      for (BooleanSupplier edge : graph.outgoingEdgesOf(currentCommand)) {
        if (edge.getAsBoolean()) {
          Command next = graph.getEdgeTarget(edge);
          next.schedule();
          currentCommand = next;
          return;
        }
      }
      // no condition was met, idle
    }
  }

  public boolean isFinished() {
    if (currentCommand == null) {
      return true;
    }

    boolean done = shouldAdvanceCurrent;
    boolean terminal = graph.outgoingEdgesOf(currentCommand).isEmpty();
    return done && terminal;
  }

  public Command toCommand() {
    return new DecisionTreeCommand(this);
  }
}
