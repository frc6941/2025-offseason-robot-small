package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public abstract class AutoRoutine {
  protected final String name;

  public AutoRoutine(String name) {
    this.name = name;
  }

  public final String getName() {
    return name;
  }

  public abstract Command getAutoCommand();

  public Command getOnSelectCommand() {
    return Commands.none();
  }

  ;
}
