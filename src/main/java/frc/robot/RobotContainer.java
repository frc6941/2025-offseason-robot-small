package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSim;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class RobotContainer {
    // Subsystems
    Swerve swerve;

    // controllers
    CommandXboxController driverController = new CommandXboxController(Constants.Controller.kDriver);
    CommandGenericHID operatorController = new CommandGenericHID(Constants.Controller.kOperator);

    public RobotContainer() {
        if (Robot.isSimulation()) {
            swerve = new Swerve(
                    Constants.Swerve.kSimConfig,
                    new ImuIOSim(),
                    new SwerveModuleIOSim(Constants.Swerve.kSimConfig),
                    new SwerveModuleIOSim(Constants.Swerve.kSimConfig),
                    new SwerveModuleIOSim(Constants.Swerve.kSimConfig),
                    new SwerveModuleIOSim(Constants.Swerve.kSimConfig)
            );
        }

        configBindings();
    }

    private void configBindings() {
        swerve.setDefaultCommand(
                SwerveCommands.driveWithJoystick(
                        swerve,
                        () -> driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> driverController.getRightX(),
                        () -> driverController.leftStick().getAsBoolean(),
                        MetersPerSecond.of(0.05),
                        DegreesPerSecond.of(5.0)
                )
        );
    }
}
