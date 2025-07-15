package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.routines.Forward0CoralAuto;
import frc.robot.auto.routines.LeftStationIntakeAuto;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.beambreak.BeambreakIOReal;
import frc.robot.subsystems.beambreak.BeambreakIOSim;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorIOARGB;
import frc.robot.subsystems.indicator.IndicatorIOSim;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.photonvision.PhotonVisionIOReal;
import frc.robot.subsystems.photonvision.PhotonVisionIOSim;
import frc.robot.subsystems.photonvision.PhotonVisionSubsystem;
import frc.robot.subsystems.roller.RollerIOReal;
import frc.robot.subsystems.roller.RollerIOSim;
import lib.ironpulse.rbd.TransformRecorder;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSim;
import lib.ironpulse.swerve.sjtu6.ImuIOPigeon;
import lib.ironpulse.swerve.sjtu6.SwerveModuleIOSJTU6;

import static edu.wpi.first.units.Units.*;


public class RobotContainer {
    // Subsystems
    Swerve swerve;
    EndEffectorSubsystem endEffectorSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    IndicatorSubsystem indicatorSubsystem;
    DestinationSupplier destinationSupplier = DestinationSupplier.getInstance();
    PhotonVisionSubsystem photonVisionSubsystem;

    // controllers
    CommandXboxController manualController = new CommandXboxController(Constants.Controller.kManual);
    CommandXboxController autoController = new CommandXboxController(Constants.Controller.kAuto);

    public RobotContainer() {
        configSubsystems();
        configAutos();
        configBindings();
    }

    private void configAutos() {
        // How corals are labeled in auto:
        //      G H
        //    I     F
        //  J         E
        //  K         D
        //    L     C
        //      A B
        //  Driver Station
        AutoActions.init(swerve, indicatorSubsystem, elevatorSubsystem, endEffectorSubsystem);

        AutoSelector.getInstance().registerAuto("Forward0CoralAuto", new Forward0CoralAuto());
        AutoSelector.getInstance().registerAuto("LeftStationIntakeAuto", new LeftStationIntakeAuto());
    }

    private void configSubsystems() {
        if (RobotBase.isReal()) {
            // Real hardware initialization
            swerve = new Swerve(
                    Constants.Swerve.kRealConfig,
                    new ImuIOPigeon(Constants.Swerve.kRealConfig),
                    new SwerveModuleIOSJTU6(Constants.Swerve.kRealConfig, 0),
                    new SwerveModuleIOSJTU6(Constants.Swerve.kRealConfig, 1),
                    new SwerveModuleIOSJTU6(Constants.Swerve.kRealConfig, 2),
                    new SwerveModuleIOSJTU6(Constants.Swerve.kRealConfig, 3));
            indicatorSubsystem = new IndicatorSubsystem(new IndicatorIOARGB());
            elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOReal());
            endEffectorSubsystem = new EndEffectorSubsystem(
                    new RollerIOReal(
                            Constants.EndEffector.MOTOR_ID,
                            Constants.CANIVORE_CAN_BUS_NAME,
                            Constants.EndEffector.STATOR_CURRENT_LIMIT_AMPS,
                            Constants.EndEffector.SUPPLY_CURRENT_LIMIT_AMPS,
                            Constants.EndEffector.IS_INVERT,
                            Constants.EndEffector.IS_BRAKE),
                    new BeambreakIOReal(2),
                    new BeambreakIOReal(0));
            photonVisionSubsystem = new PhotonVisionSubsystem(
                    new PhotonVisionIOReal(0)//,
                    //new PhotonVisionIOReal(1)
            );
        } else {
            swerve = new Swerve(
                    Constants.Swerve.kSimConfig,
                    new ImuIOSim(),
                    new SwerveModuleIOSim(Constants.Swerve.kSimConfig, 0),
                    new SwerveModuleIOSim(Constants.Swerve.kSimConfig, 1),
                    new SwerveModuleIOSim(Constants.Swerve.kSimConfig, 2),
                    new SwerveModuleIOSim(Constants.Swerve.kSimConfig, 3));
            indicatorSubsystem = new IndicatorSubsystem(new IndicatorIOSim());
            elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
            photonVisionSubsystem = new PhotonVisionSubsystem(
                    new PhotonVisionIOSim(0),
                    new PhotonVisionIOSim(1)
            );
            endEffectorSubsystem = new EndEffectorSubsystem(
                    //TODO: change
                    new RollerIOSim(1, Constants.EndEffector.ROLLER_RATIO,
                            new SimpleMotorFeedforward(0.0, 0.24),
                            new ProfiledPIDController(0.5, 0.0, 0.0,
                                    new TrapezoidProfile.Constraints(15, 1))),
                    new BeambreakIOSim(3),
                    new BeambreakIOSim(2));
        }
    }

    private void configBindings() {
        swerve.setDefaultCommand(
                SwerveCommands.driveWithJoystick(
                        swerve,
                        () -> greaterInput(-manualController.getLeftY(), -autoController.getLeftY()),
                        () -> greaterInput(-manualController.getLeftX(), -autoController.getLeftX()),
                        () -> greaterInput(-manualController.getRightX(), -autoController.getRightX()),
                        RobotStateRecorder::getPoseDriverRobotCurrent,
                        MetersPerSecond.of(0.04),
                        DegreesPerSecond.of(3.0)));
        manualController.start().onTrue(
                SwerveCommands.resetAngle(swerve, new Rotation2d())
                        .alongWith(Commands.runOnce(
                                () -> {
                                    RobotStateRecorder.getInstance().resetTransform(
                                            TransformRecorder.kFrameWorld,
                                            TransformRecorder.kFrameRobot);
                                    indicatorSubsystem.setPattern(IndicatorIO.Patterns.RESET_ODOM);
                                })));
        manualController.povDown().onTrue(elevatorSubsystem.zeroElevator());
        manualController.rightBumper().onTrue(new IntakeCommand(elevatorSubsystem, endEffectorSubsystem, indicatorSubsystem));//手动intake
        manualController.rightTrigger().whileTrue(new ShootCommand(endEffectorSubsystem, indicatorSubsystem));//强制放
        manualController.povLeft().whileTrue(Commands.sequence(
                        Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L2)),
                        Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(destinationSupplier.getElevatorSetpoint(true)))))
                .onFalse(new IntakeCommand(elevatorSubsystem, endEffectorSubsystem, indicatorSubsystem
                ));
        manualController.leftTrigger().whileTrue(Commands.sequence(
                        Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L3)),
                        Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(destinationSupplier.getElevatorSetpoint(true)))))
                .onFalse(new IntakeCommand(elevatorSubsystem, endEffectorSubsystem, indicatorSubsystem));
        manualController.leftBumper().whileTrue(Commands.sequence(
                        Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L4)),
                        Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(destinationSupplier.getElevatorSetpoint(true)))))
                .onFalse(new IntakeCommand(elevatorSubsystem, endEffectorSubsystem, indicatorSubsystem));
        manualController.y().onTrue(elevatorSubsystem.zeroElevator());//电梯归零

        autoController.start().onTrue(
                SwerveCommands.resetAngle(swerve, new Rotation2d())
                        .alongWith(Commands.runOnce(
                                () -> {
                                    RobotStateRecorder.getInstance().resetTransform(
                                            TransformRecorder.kFrameWorld,
                                            TransformRecorder.kFrameRobot);
                                    indicatorSubsystem.setPattern(IndicatorIO.Patterns.RESET_ODOM);
                                })));
        autoController.rightBumper().whileTrue(new AutoIntakeCommand(swerve, endEffectorSubsystem, elevatorSubsystem, indicatorSubsystem));//手动intake
        autoController.rightTrigger().whileTrue(new AutoShootCommand(swerve, indicatorSubsystem, elevatorSubsystem, endEffectorSubsystem, () -> false));//强制放

        autoController.leftStick().onTrue(Commands.runOnce(() -> DestinationSupplier.getInstance().updateBranch(false)));
        autoController.rightStick().onTrue(Commands.runOnce(() -> DestinationSupplier.getInstance().updateBranch(true)));

        autoController.povLeft().whileTrue(new IntakeCommand(elevatorSubsystem, endEffectorSubsystem, indicatorSubsystem));
        autoController.povRight().whileTrue(new AutoIntakeCommand(swerve, endEffectorSubsystem, elevatorSubsystem, indicatorSubsystem));

        autoController.y().onTrue(elevatorSubsystem.zeroElevator());

        autoController.leftTrigger().whileTrue(
                Commands.sequence(
                        Commands.runOnce(() -> DestinationSupplier.getInstance().updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L2)),
                        new AutoShootCommand(swerve, indicatorSubsystem, elevatorSubsystem, endEffectorSubsystem, autoController.rightTrigger())
                )
        );
        autoController.leftBumper().whileTrue(
                Commands.sequence(
                        Commands.runOnce(() -> DestinationSupplier.getInstance().updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L3)),
                        new AutoShootCommand(swerve, indicatorSubsystem, elevatorSubsystem, endEffectorSubsystem, autoController.rightTrigger())
                )
        );
        autoController.rightBumper().whileTrue(
                Commands.sequence(
                        Commands.runOnce(() -> DestinationSupplier.getInstance().updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L4)),
                        new AutoShootCommand(swerve, indicatorSubsystem, elevatorSubsystem, endEffectorSubsystem, autoController.rightTrigger())
                )
        );
    }

    public void robotPeriodic() {
        if (photonVisionSubsystem.estimatedPose != null) {
            swerve.addVisionMeasurement(
                    photonVisionSubsystem.estimatedPose,
                    photonVisionSubsystem.timestampSeconds,
                    VecBuilder.fill(0.1, 0.1, 0.3, Double.MAX_VALUE));
        }
        var now = Seconds.of(Timer.getTimestamp());
        RobotStateRecorder.getInstance().putTransform(
                swerve.getEstimatedPose(), now,
                TransformRecorder.kFrameWorld,
                TransformRecorder.kFrameRobot);
        RobotStateRecorder.putVelocityRobot(now, swerve.getChassisSpeeds());
        RobotStateRecorder.periodic();
    }

    private double greaterInput(double input1, double input2) {
        if (Math.abs(input1) > Math.abs(input2)) return input1;
        return input2;
    }
}
