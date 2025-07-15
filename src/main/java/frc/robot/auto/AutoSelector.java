package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lib.ironpulse.utils.Logging;

public class AutoSelector {
  private static AutoSelector instance;
  private final SendableChooser<AutoRoutine> autoSelector;

  private AutoSelector() {
    autoSelector = new SendableChooser<>();
    autoSelector.setDefaultOption("None", new AutoRoutine("None") {
      @Override
      public Command getAutoCommand() {
        return Commands.none();
      }
    });
    autoSelector.onChange((routine) -> {
      Logging.info("AutoSelector", "New AutoRoutine selected: %s", routine.getName());
      routine.getOnSelectCommand().schedule();
    });
    SmartDashboard.putData("Auto Selector", autoSelector);
  }

  public static AutoSelector getInstance() {
    if (instance == null) {
      instance = new AutoSelector();
    }
    return instance;
  }

  public void registerAuto(String name, AutoRoutine auto) {
    autoSelector.addOption(name, auto);
  }

  public Command getAutoCommand() {
    return autoSelector.getSelected().getAutoCommand(); // this will refresh auto on selection
  }
}
