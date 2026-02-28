// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight;

public class drivetrainaprtagtrack extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Limelight limelight;
    private final PIDController pid = new PIDController(0.025, 0.0, 0.002);
    private final double toleranceDeg = 1.0;
    private final double maxTurn = 0.5; // max rotational rate (rad/s)

    // mutable container for the rotational rate used by the supplier
    private final double[] rotRef = new double[1];
    private Command applyRequestCmd;

    public drivetrainaprtagtrack(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(drivetrain);
        pid.setTolerance(toleranceDeg);
        pid.enableContinuousInput(-180.0, 180.0); // safe for angles
    }

    @Override
    public void initialize() {
        pid.reset();
        pid.setSetpoint(0.0);
        rotRef[0] = 0.0;

        Supplier<SwerveRequest> supplier = () -> new SwerveRequest.FieldCentric()
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(rotRef[0]);

        applyRequestCmd = drivetrain.applyRequest(supplier);
        applyRequestCmd.schedule();
    }

    @Override
    public void execute() {
        if (!limelight.hasValidTarget()) {
            rotRef[0] = 0.0;
            return;
        }
        double tx = limelight.getTX(); // horizontal offset in degrees (Limelight uses getTX/getTY)
        double rotation = pid.calculate(tx);
        rotation = Math.max(-maxTurn, Math.min(maxTurn, rotation));
        rotRef[0] = rotation; // treated as rad/s (small value)
    }

    @Override
    public boolean isFinished() {
        return limelight.hasValidTarget() && pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        if (applyRequestCmd != null) {
            applyRequestCmd.cancel();
        }
        // ensure drivetrain stops rotating
        drivetrain.applyRequest(() -> new SwerveRequest.FieldCentric()
                .withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0))
            .schedule();
    }
}
