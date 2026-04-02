// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.spindexer.spindexer;
import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.turret.turret;
import frc.robot.subsystems.intake.intake;

import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.states;

/** Add your docs here. */
public class AutoRoutines {
    private final AutoFactory m_factory;
    private turret turret;
    private shooter shooter;
    private spindexer spindexer;
    private intake intake;
    private Limelight shooterLL = new Limelight("limelight-shooter", 1, 0, 0, 0, false);
    public AutoRoutines(AutoFactory factory,turret turret, shooter shooter, spindexer spindexer, intake intake, Limelight shooterLL) {
        m_factory = factory;
        this.turret = turret;
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.intake = intake;
        this.shooterLL = shooterLL;
    }
    public AutoRoutine RightAuto(){

        final AutoRoutine routine = m_factory.newRoutine("RightAuto");
        final AutoTrajectory RightAutoShootFirst = routine.trajectory("RightAutoShootFirst");
        final AutoTrajectory RightAutoGoGetFuel1 = routine.trajectory("RightAutoGoGetFuel1");

        routine.active().onTrue(
            Commands.sequence(

            RightAutoShootFirst.resetOdometry(),
            Commands.parallel(
                RightAutoShootFirst.cmd(),
                    shooter.setStateCommand(states.shooterState.SHOOTING)
            )
            )
        );

        RightAutoShootFirst.done().onTrue(
            Commands.sequence(
                new WaitCommand(2),
                turret.setStateCommand(states.turretspinState.TRACKING),
                new WaitCommand(3),
                spindexer.setStateCommand(states.spindexState.RUNNING),
                new WaitCommand(2),
                RightAutoGoGetFuel1.cmd()

            )
            );

        return routine;

    }
    public AutoRoutine CenterAuto(){

        final AutoRoutine routine = m_factory.newRoutine("CenterAuto");
        final AutoTrajectory CenterAutoShootFirst = routine.trajectory("CenterAutoShootFirst");
        final AutoTrajectory CenterAutoGoHome = routine.trajectory("CenterAutoGoHome");

        routine.active().onTrue(
            Commands.sequence(

            CenterAutoShootFirst.resetOdometry(),
            Commands.parallel(
                CenterAutoShootFirst.cmd(),
                    shooter.setStateCommand(states.shooterState.SHOOTING)
            )
            )
        );

        CenterAutoShootFirst.done().onTrue(
            Commands.sequence(
                new WaitCommand(1),
                turret.setStateCommand(states.turretspinState.TRACKING),
                new WaitCommand(1),
                spindexer.setStateCommand(states.spindexState.RUNNING),
                new WaitCommand(2),
                turret.setStateCommand(states.turretspinState.TRACKING),
                CenterAutoGoHome.cmd(),
                spindexer.setStateCommand(states.spindexState.IDLE),
                shooter.setStateCommand(states.shooterState.IDLE)
            )
            );

        return routine;

    }
    public AutoRoutine LeftAuto(){

        final AutoRoutine routine = m_factory.newRoutine("LeftAuto");
        final AutoTrajectory LeftAutoShootFirst = routine.trajectory("LeftAutoShootFirst");
        final AutoTrajectory LeftAutoPickUp1 = routine.trajectory("CenterAutoGoHome");

        routine.active().onTrue(
            Commands.sequence(

            LeftAutoPickUp1.resetOdometry(),
            Commands.parallel(
                LeftAutoPickUp1.cmd(),
                    shooter.setStateCommand(states.shooterState.SHOOTING)
            )
            )
        );

        return routine;

    }
}
