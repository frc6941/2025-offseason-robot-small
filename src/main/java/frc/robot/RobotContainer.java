package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lib.ironpulse.rbd.TransformRecorder;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSim;

import static edu.wpi.first.units.Units.*;

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
                        () -> RobotStateRecorder.getInstance().getTransform(
                                Seconds.of(Timer.getTimestamp()),
                                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(
                                        DriverStation.Alliance.Blue)
                                        ? RobotStateRecorder.kFrameDriverStationBlue :
                                        RobotStateRecorder.kFrameDriverStationRed,
                                TransformRecorder.kFrameRobot
                        ).orElse(new Pose3d()),
                        MetersPerSecond.of(0.05),
                        DegreesPerSecond.of(5.0)
                )
        );
    }
}
