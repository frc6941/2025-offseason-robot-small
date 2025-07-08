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
import lib.ironpulse.swerve.sjtu6.ImuIOPigeon;
import lib.ironpulse.swerve.sjtu6.SwerveModuleIOSJTU6;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Elevator.motorNum;

import java.text.BreakIterator;

public class RobotContainer {
    // Subsystems
    Swerve swerve;

    // controllers
    CommandXboxController driverController = new CommandXboxController(Constants.Controller.kDriver);
    CommandGenericHID operatorController = new CommandGenericHID(Constants.Controller.kOperator);

    public RobotContainer() {
        
        // if (motorNum%2!= 0) {
        //     throw new IllegalCallerException("Motor number must be even");
        // }
        switch (Constants.kRobotType) {
            case PRAC -> {
                swerve = new Swerve(
                        Constants.Swerve.kRealConfig, new ImuIOPigeon(Constants.Swerve.kRealConfig),
                        new SwerveModuleIOSJTU6(Constants.Swerve.kRealConfig, 0),
                        new SwerveModuleIOSJTU6(Constants.Swerve.kRealConfig, 1),
                        new SwerveModuleIOSJTU6(Constants.Swerve.kRealConfig, 2),
                        new SwerveModuleIOSJTU6(Constants.Swerve.kRealConfig, 3)
                );
                SwerveModuleIOSJTU6.startSyncThread();
            }
            case SIM -> {
                swerve = new Swerve(
                        Constants.Swerve.kSimConfig, new ImuIOSim(),
                        new SwerveModuleIOSim(Constants.Swerve.kSimConfig, 0),
                        new SwerveModuleIOSim(Constants.Swerve.kSimConfig, 1),
                        new SwerveModuleIOSim(Constants.Swerve.kSimConfig, 2),
                        new SwerveModuleIOSim(Constants.Swerve.kSimConfig, 3)
                );
            }
        }

        configBindings();
    }

    private void configBindings() {
        swerve.setDefaultCommand(SwerveCommands.driveWithJoystick(
            swerve, 
            () -> driverController.getLeftY(), 
            () -> driverController.getLeftX(),
            () -> driverController.getRightX(), 
            //TODO: not working
            () -> RobotStateRecorder.getInstance().getTransform(
                Seconds.of(Timer.getTimestamp()),
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(
                    DriverStation.Alliance.Blue) ? RobotStateRecorder.kFrameDriverStationBlue 
                    : RobotStateRecorder.kFrameDriverStationRed,
                TransformRecorder.kFrameRobot
            ).orElse(new Pose3d()), 
            MetersPerSecond.of(0.05), 
            DegreesPerSecond.of(5.0)
        ));
    }

    public void robotPeriodic() {
        var now = Seconds.of(Timer.getTimestamp());
        swerve.getEstimatedPositionAt(now).ifPresent(
                pose -> RobotStateRecorder.getInstance().putTransform(
                        pose, now, RobotStateRecorder.kFrameWorld, RobotStateRecorder.kFrameRobot));
    }
}
