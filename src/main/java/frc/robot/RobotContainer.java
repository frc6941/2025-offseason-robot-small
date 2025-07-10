package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.indicator.IndicatorIOARGB;
import frc.robot.subsystems.indicator.IndicatorIOSim;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.limelight.LimelightIOReal;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.roller.RollerIOReal;
import frc.robot.subsystems.roller.RollerIOSim;
import lib.ironpulse.rbd.TransformRecorder;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSim;
import lib.ironpulse.swerve.sjtu6.ImuIOPigeon;
import lib.ironpulse.swerve.sjtu6.SwerveModuleIOSJTU6;

import java.util.HashMap;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Limelight.LIMELIGHT_LEFT;
import static frc.robot.Constants.Limelight.LIMELIGHT_RIGHT;


public class RobotContainer {
    // Subsystems
    Swerve swerve;
    EndEffectorSubsystem endEffectorSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    LimelightSubsystem limelightSubsystem;
    IndicatorSubsystem indicatorSubsystem;
    //PhotonvisionSubsystem photonvisionSubsystem

    // controllers
    CommandXboxController driverController = new CommandXboxController(Constants.Controller.kDriver);
    CommandGenericHID operatorController = new CommandGenericHID(Constants.Controller.kOperator);

    public RobotContainer() {
        configSubsystems();
        configBindings();
    }

    private void configSubsystems() {
//        switch (Constants.kRobotType) {
//            case PRAC -> {
//                swerve = new Swerve(
//                        Constants.Swerve.kRealConfig, new ImuIOPigeon(Constants.Swerve.kRealConfig),
//                        new SwerveModuleIOSJTU6(Constants.Swerve.kRealConfig, 0),
//                        new SwerveModuleIOSJTU6(Constants.Swerve.kRealConfig, 1),
//                        new SwerveModuleIOSJTU6(Constants.Swerve.kRealConfig, 2),
//                        new SwerveModuleIOSJTU6(Constants.Swerve.kRealConfig, 3)
//                );
//                SwerveModuleIOSJTU6.startSyncThread();
//            }
//            case SIM -> {
//                swerve = new Swerve(
//                        Constants.Swerve.kSimConfig, new ImuIOSim(),
//                        new SwerveModuleIOSim(Constants.Swerve.kSimConfig, 0),
//                        new SwerveModuleIOSim(Constants.Swerve.kSimConfig, 1),
//                        new SwerveModuleIOSim(Constants.Swerve.kSimConfig, 2),
//                        new SwerveModuleIOSim(Constants.Swerve.kSimConfig, 3)
//                );
//            }
//        }
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
                            Constants.EndEffector.IS_BRAKE));
            limelightSubsystem = new LimelightSubsystem(new HashMap<>() {
                {
                    put(LIMELIGHT_LEFT, new LimelightIOReal(LIMELIGHT_LEFT));
                    put(LIMELIGHT_RIGHT, new LimelightIOReal(LIMELIGHT_RIGHT));
                }
            });
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
            limelightSubsystem = new LimelightSubsystem(new HashMap<>() {
                {
                    put(LIMELIGHT_LEFT, new LimelightIOReal(LIMELIGHT_LEFT));
                    put(LIMELIGHT_RIGHT, new LimelightIOReal(LIMELIGHT_RIGHT));
                }
            });
            endEffectorSubsystem = new EndEffectorSubsystem(
                    //TODO: change
                    new RollerIOSim(1, Constants.EndEffector.ROLLER_RATIO,
                            new SimpleMotorFeedforward(0.0, 0.24),
                            new ProfiledPIDController(0.5, 0.0, 0.0,
                                    new TrapezoidProfile.Constraints(15, 1))));
            //photonVisionSubsystem = new PhotonVisionSubsystem(new PhotonVisionIOSim(0));
        }
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
