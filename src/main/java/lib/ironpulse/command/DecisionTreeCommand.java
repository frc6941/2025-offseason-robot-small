package lib.ironpulse.command;

import edu.wpi.first.wpilibj2.command.Command;


public class DecisionTreeCommand extends Command {
  private final DecisionTree tree;

  public DecisionTreeCommand(DecisionTree tree) {
    this.tree = tree;
  }

  @Override
  public void initialize() {
    tree.reset();
  }

  @Override
  public void execute() {
    tree.tick();
  }

  @Override
  public boolean isFinished() {
    return tree.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Command current = tree.getCurrentCommand();
      if (current != null && current.isScheduled()) current.cancel();
    }
  }
}