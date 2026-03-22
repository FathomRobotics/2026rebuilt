// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.states;
import frc.robot.subsystems.intake.intake;
import frc.robot.util.HubTracker;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    public final intake intake = new intake();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {

        double startTime = Timer.getFPGATimestamp();
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    SmartDashboard.putNumber(
        "HubTracker/Time Until Shift",
        HubTracker.timeRemainingInCurrentShift().orElse(Seconds.of(0)).in(Seconds));
    SmartDashboard.putBoolean(
        "HubTracker/RedWonAuto", HubTracker.getAutoWinner().orElse(Alliance.Blue) == Alliance.Red);
    SmartDashboard.putBoolean("HubTracker/GameDataPresent", !HubTracker.getAutoWinner().isEmpty());

    SmartDashboard.putNumber(
        "HubTracker/TimeUtilActive",
        HubTracker.timeUntilActive().orElse(Seconds.of(0)).in(Seconds));

    double codeRuntime = (Timer.getFPGATimestamp() - startTime) * 1000.0;
    SmartDashboard.putNumber("Code Runtime (ms)", codeRuntime);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        this.intake.setState(states.intakeState.IDLE);

    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
