package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.AutoActions;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.routines.*;
import frc.robot.commands.*;
import frc.robot.display.Display;
import frc.robot.drivers.DestinationSupplier;
import frc.robot.subsystems.ElevatorSetPoint;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SuperstructureState;
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
import lib.ironpulse.swerve.sim.SwerveModuleIOSimpleSim;
import lib.ironpulse.swerve.sjtu6.ImuIOPigeon;
import lib.ironpulse.swerve.sjtu6.SwerveModuleIOSJTU6;
import lib.ironpulse.command.DriverConditionalCommand;

import static edu.wpi.first.units.Units.*;


public class RobotContainer {
    // Subsystems
    Swerve swerve;
    EndEffectorSubsystem endEffectorSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    IndicatorSubsystem indicatorSubsystem;
    DestinationSupplier destinationSupplier = DestinationSupplier.getInstance();
    PhotonVisionSubsystem photonVisionSubsystem;
    Display display = Display.getInstance();
    Superstructure superstructure;

    // controllers
    CommandXboxController driverController = new CommandXboxController(Constants.Controller.kDriver);
    CommandXboxController testerController = new CommandXboxController(Constants.Controller.kTest);

    public RobotContainer() {
        configSubsystems();
        configAutos();
        configBindings();
    }

    private void configAutos() {
        // How corals are labeled in auto:
        //      H G
        //    I     F
        //  J         E
        //  K         D
        //    L     C
        //      A B
        //  Driver Station
        AutoActions.init(swerve, indicatorSubsystem, elevatorSubsystem, endEffectorSubsystem,superstructure);

        AutoSelector.getInstance().registerAuto("Forward0CoralAuto", new Forward0CoralAuto());
        AutoSelector.getInstance().registerAuto("Left5L4Auto", new Left5L4Auto());
        AutoSelector.getInstance().registerAuto("Right5L4Auto", new Right5L4Auto());
        AutoSelector.getInstance().registerAuto("Left5L3Auto", new Left5L3Auto());
        AutoSelector.getInstance().registerAuto("Right5L3Auto", new Right5L3Auto());
        AutoSelector.getInstance().registerAuto("Middle1CoralAuto", new Middle1CoralAuto());
        AutoSelector.getInstance().registerAuto("TestAuto", new TestAuto());
        AutoSelector.getInstance().registerAuto("Left3L4And2L3Auto", new Left3L4And2L3Auto());
        AutoSelector.getInstance().registerAuto("Right3L4And2L3Auto", new Right3L4And2L3Auto());
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
                    new PhotonVisionIOReal(0),
                    new PhotonVisionIOReal(1)
            );
            superstructure = new Superstructure(endEffectorSubsystem, elevatorSubsystem);
        } else {
            swerve = new Swerve(
                    Constants.Swerve.kSimConfig,
                    new ImuIOSim(),
                    new SwerveModuleIOSimpleSim(Constants.Swerve.kSimConfig, 0),
                    new SwerveModuleIOSimpleSim(Constants.Swerve.kSimConfig, 1),
                    new SwerveModuleIOSimpleSim(Constants.Swerve.kSimConfig, 2),
                    new SwerveModuleIOSimpleSim(Constants.Swerve.kSimConfig, 3));
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
            superstructure = new Superstructure(endEffectorSubsystem,elevatorSubsystem);
        }
    }

    private void configBindings() {
        swerve.setDefaultCommand(
                SwerveCommands.driveWithJoystick(
                        swerve,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        RobotStateRecorder::getPoseDriverRobotCurrent,
                        MetersPerSecond.of(0.04),
                        DegreesPerSecond.of(3.0)));

        driverController.start().onTrue(
                SwerveCommands.resetAngle(swerve, new Rotation2d())
                        .alongWith(Commands.runOnce(
                                () -> {
                                    RobotStateRecorder.getInstance().resetTransform(
                                            TransformRecorder.kFrameWorld,
                                            TransformRecorder.kFrameRobot);
                                    indicatorSubsystem.setPattern(IndicatorIO.Patterns.RESET_ODOM);
                                })).ignoringDisable(true));
        driverController.povRight().whileTrue(
                new ConditionalCommand(
                        Commands.sequence(
                                Commands.runOnce(() -> DestinationSupplier.getInstance().updateBranch(true)),
                                new AutoShootCommand(swerve, indicatorSubsystem, elevatorSubsystem, endEffectorSubsystem, superstructure, ElevatorSetPoint.L2,driverController.b())),
                        superstructure.runGoal(()->SuperstructureState.L2),
                        DestinationSupplier.getInstance()::isAuto
                )
        );//背键-右侧靠内
        driverController.povLeft().whileTrue(
                new ConditionalCommand(
                        Commands.sequence(
                                Commands.runOnce(() -> DestinationSupplier.getInstance().updateBranch(false)),
//                                Commands.runOnce(() -> DestinationSupplier.getInstance().updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L2)),
                                new AutoShootCommand(swerve, indicatorSubsystem, elevatorSubsystem, endEffectorSubsystem, superstructure, ElevatorSetPoint.L2,driverController.b())),
                        superstructure.runGoal(()->SuperstructureState.L2),
                        DestinationSupplier.getInstance()::isAuto
                )
        );//背键-左侧靠内

        driverController.rightTrigger().whileTrue(
                new ConditionalCommand(
                        Commands.sequence(
                                Commands.runOnce(() -> DestinationSupplier.getInstance().updateBranch(true)),
//                                Commands.runOnce(() -> DestinationSupplier.getInstance().updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L3)),
                                new AutoShootCommand(swerve, indicatorSubsystem, elevatorSubsystem, endEffectorSubsystem,superstructure,ElevatorSetPoint.L3, driverController.b())),
                        superstructure.runGoal(()->SuperstructureState.L3),
                        DestinationSupplier.getInstance()::isAuto
                )
        );
        driverController.leftTrigger().whileTrue(
                new ConditionalCommand(
                        Commands.sequence(
                                Commands.runOnce(() -> DestinationSupplier.getInstance().updateBranch(false)),
//                                Commands.runOnce(() -> DestinationSupplier.getInstance().updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L3)),
                                new AutoShootCommand(swerve, indicatorSubsystem, elevatorSubsystem, endEffectorSubsystem, superstructure, ElevatorSetPoint.L3, driverController.b())),
//                        Commands.runOnce(() -> DestinationSupplier.getInstance().updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L3)),
                        superstructure.runGoal(()->SuperstructureState.L3),
                        DestinationSupplier.getInstance()::isAuto
                )
        );

        driverController.rightBumper().whileTrue(
                new ConditionalCommand(
                        Commands.sequence(
                                Commands.runOnce(() -> DestinationSupplier.getInstance().updateBranch(true)),
//                                Commands.runOnce(() -> DestinationSupplier.getInstance().updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L4)),
                                new AutoShootCommand(swerve, indicatorSubsystem, elevatorSubsystem, endEffectorSubsystem, superstructure, ElevatorSetPoint.L4, driverController.b())),
//                        Commands.runOnce(() -> DestinationSupplier.getInstance().updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L4)),
                        superstructure.runGoal(()->SuperstructureState.L4),
                        DestinationSupplier.getInstance()::isAuto

                )
        );
        driverController.leftBumper().whileTrue(
                new ConditionalCommand(
                        Commands.sequence(
                                Commands.runOnce(() -> DestinationSupplier.getInstance().updateBranch(false)),
//                                Commands.runOnce(() -> DestinationSupplier.getInstance().updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L4)),
                                new AutoShootCommand(swerve, indicatorSubsystem, elevatorSubsystem, endEffectorSubsystem, superstructure, ElevatorSetPoint.L4, driverController.b())),
//                        Commands.runOnce(() -> DestinationSupplier.getInstance().updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L4)),
                        superstructure.runGoal(()->SuperstructureState.L4),
                        DestinationSupplier.getInstance()::isAuto
                )
        );

        driverController.povUp().onTrue(Commands.runOnce(() -> DestinationSupplier.getInstance().setIsAuto(!DestinationSupplier.getInstance().isAuto())).ignoringDisable(true));
//        driverController.povRight().onTrue(superstructure.runGoal(()->SuperstructureState.L2));//背键-右侧靠内
//        driverController.povLeft().onTrue(superstructure.runGoal(()->SuperstructureState.L2));//背键-左侧靠内
//        driverController.rightTrigger().onTrue(superstructure.runGoal(()->SuperstructureState.L3));
//        driverController.leftTrigger().onTrue(superstructure.runGoal(()->SuperstructureState.L3));
//        driverController.rightBumper().onTrue(superstructure.runGoal(()->SuperstructureState.L4));
//        driverController.leftBumper().onTrue(superstructure.runGoal(()->SuperstructureState.L4));


        driverController.y().toggleOnTrue(Commands.sequence(
                superstructure.runGoal(()->SuperstructureState.P2)
        ));
        driverController.a().toggleOnTrue(Commands.sequence(
                superstructure.runGoal(()->SuperstructureState.P1)
        ));

        driverController.b().whileTrue(superstructure.shoot());
        driverController.x().toggleOnTrue(superstructure.runGoal(()-> SuperstructureState.INTAKE).until(endEffectorSubsystem::isFrontEE));

        driverController.povDown().onTrue(elevatorSubsystem.zeroElevator());

        driverController.povDownLeft().onTrue(superstructure.runGoal(SuperstructureState.IDLE));
        driverController.povDownRight().onTrue(superstructure.runGoal(SuperstructureState.OUTTAKE));

        driverController.leftStick().whileTrue(
                new DriverConditionalCommand(
                        new NavToStationCommand(swerve, indicatorSubsystem),
                        Commands.run(() -> elevatorSubsystem.setElevatorPosition(DestinationSupplier.getInstance().getElevatorSetpoint(true)))
                                .finallyDo(() -> elevatorSubsystem.setElevatorPosition(0)),
                        DestinationSupplier.getInstance()::isAuto
                )
        );//背键-左侧靠外
        driverController.rightStick().whileTrue(
                new DriverConditionalCommand(
                        new NavToStationCommand(swerve, indicatorSubsystem),
                        Commands.run(() -> elevatorSubsystem.setElevatorPosition(DestinationSupplier.getInstance().getElevatorSetpoint(true)))
                                .finallyDo(() -> elevatorSubsystem.setElevatorPosition(0)),
                        DestinationSupplier.getInstance()::isAuto
                )
        );//背键-右侧靠外

        testerController.rightBumper().whileTrue(Commands.sequence(                AutoActions.autoScore('E',ElevatorSetPoint.L4)));
        testerController.b().whileTrue(superstructure.runGoal(()->SuperstructureState.P1));
        testerController.a().whileTrue(superstructure.runGoal(()->SuperstructureState.L2_EJECT
        ));
        testerController.x().whileTrue(superstructure.runGoal(()->SuperstructureState.L2));
        testerController.y().whileTrue(superstructure.runGoal(()->SuperstructureState.INTAKE));
    }

    public void robotPeriodic() {
        if (photonVisionSubsystem.estimatedPose != null) {
            swerve.addVisionMeasurement(
                    photonVisionSubsystem.estimatedPose,
                    photonVisionSubsystem.timestampSeconds,
                    VecBuilder.fill(0.1, 0.1, 0.3, Double.MAX_VALUE));
        }
        display.update();
        var now = Seconds.of(Timer.getTimestamp());
        RobotStateRecorder.getInstance().putTransform(
                swerve.getEstimatedPose(), now,
                TransformRecorder.kFrameWorld,
                TransformRecorder.kFrameRobot);
        RobotStateRecorder.putVelocityRobot(now, swerve.getChassisSpeeds());
        RobotStateRecorder.periodic();
    }
}
