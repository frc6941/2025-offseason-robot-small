package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeCommand;
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
    CommandXboxController driverController = new CommandXboxController(Constants.Controller.kDriver);
    CommandGenericHID operatorController = new CommandGenericHID(Constants.Controller.kOperator);

    public RobotContainer() {
        configSubsystems();
        configBindings();
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
            //photonVisionSubsystem = new PhotonVisionSubsystem(new PhotonVisionIOReal(0));
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
                        () -> driverController.getLeftY(),
                        () -> driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        RobotStateRecorder::getPoseDriverRobotCurrent,
                        MetersPerSecond.of(0.04),
                        DegreesPerSecond.of(3.0)));
        driverController.start().onTrue(SwerveCommands.resetAngle(swerve, new Rotation2d())
                .alongWith(Commands.runOnce(
                        () -> {
                            RobotStateRecorder.getInstance().resetTransform(
                                    TransformRecorder.kFrameWorld,
                                    TransformRecorder.kFrameRobot);
                            indicatorSubsystem.setPattern(IndicatorIO.Patterns.RESET_ODOM);
                        })));
        driverController.leftBumper();//自动取 自动对正 到位 吸球[]\
        driverController.rightBumper();//自动放 ELEvator自动到位 强制射
        driverController.leftTrigger().whileTrue(new IntakeCommand(elevatorSubsystem,endEffectorSubsystem));//手动intake
        driverController.rightTrigger().whileTrue(Commands.runOnce(() -> endEffectorSubsystem.setRollerVoltage(EndEffectorParamsNT.CORAL_SHOOT_VOLTAGE.getValue())));//强制放
        driverController.a().onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L2)).ignoringDisable(true));
        driverController.b().onTrue(Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(destinationSupplier.getElevatorSetpoint(true))));//Elevator到位
        driverController.x().onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L3)).ignoringDisable(true));
        driverController.y().onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L4)).ignoringDisable(true));
        driverController.leftStick();//L
        driverController.rightStick();//R
        driverController.povDown().onTrue(elevatorSubsystem.zeroElevator());//电梯归零
    }

    public void robotPeriodic() {
        photonVisionSubsystem.estimatedPose.ifPresent(
                pose3d -> swerve.addVisionMeasurement(
                        pose3d,
                        photonVisionSubsystem.timestampSeconds,
                        VecBuilder.fill(0.1, 0.1, 0.3, 9999.0)));

        var now = Seconds.of(Timer.getTimestamp());
        RobotStateRecorder.getInstance().putTransform(
                swerve.getEstimatedPose(), now,
                TransformRecorder.kFrameWorld,
                TransformRecorder.kFrameRobot);
        RobotStateRecorder.putVelocityRobot(now, swerve.getChassisSpeeds());
        RobotStateRecorder.periodic();
    }
}
