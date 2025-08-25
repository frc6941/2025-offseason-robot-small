// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.AutoSelector;
import lib.ironpulse.utils.LoggedTracer;
import lib.ironpulse.utils.PhoenixUtils;
import lib.ntext.NTParameterRegistry;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
    RobotContainer container;
    private Command autonomousCommand;

    public Robot() {
        super(Constants.kDtS); // accelerate loop time
    }

    @Override
    public void robotInit() {
        // logger initialization
        Logger.addDataReceiver(new WPILOGWriter());
        if (DriverStation.getMatchType() == DriverStation.MatchType.None) {
            Logger.addDataReceiver(new NT4Publisher());
        }
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        LoggedPowerDistribution.getInstance(20, PowerDistribution.ModuleType.kRev);
        Logger.start();

        // early-stage initialization
        DriverStation.silenceJoystickConnectionWarning(true);
        PowerDistribution PDP = new PowerDistribution();
        PDP.clearStickyFaults();
        PDP.close();


        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        container = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99);
        if (Constants.kTuning)
            NTParameterRegistry.refresh(); // refresh all the parameters when needed
        PhoenixUtils.refreshAll();
        CommandScheduler.getInstance().run();
        LoggedTracer.record("Commands");
        container.robotPeriodic();
    }

    @Override
    public void autonomousInit() {
        try {
            autonomousCommand = AutoSelector.getInstance().getAutoCommand();
        } catch (Exception e) {
            System.out.println("Autonomous command failed: " + e);
            e.printStackTrace();
            autonomousCommand = null;
        }

        if (autonomousCommand != null) autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
