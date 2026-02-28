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

    // supplier that provides the translational portion of the SwerveRequest
    private final Supplier<SwerveRequest> baseSupplier;

    public drivetrainaprtagtrack(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
        this(drivetrain, limelight, null);
    }

    /**
     * @param drivetrain drivetrain instance
     * @param limelight limelight instance
     * @param baseSupplier supplier that returns a base SwerveRequest (used to keep translation). If null, translation will be zero.
     */
    public drivetrainaprtagtrack(CommandSwerveDrivetrain drivetrain, Limelight limelight, Supplier<SwerveRequest> baseSupplier) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.baseSupplier = baseSupplier;
        addRequirements(drivetrain);
        pid.setTolerance(toleranceDeg);
        pid.enableContinuousInput(-180.0, 180.0); // safe for angles
    }

    @Override
    public void initialize() {
        pid.reset();
        pid.setSetpoint(0.0);
        rotRef[0] = 0.0;

        Supplier<SwerveRequest> supplier = () -> {
            SwerveRequest base = null;
            if (baseSupplier != null) {
                try {
                    base = baseSupplier.get();
                } catch (Exception e) {
                    base = null;
                }
            }
            if (base instanceof SwerveRequest.FieldCentric) {
                return ((SwerveRequest.FieldCentric) base).withRotationalRate(rotRef[0]);
            }
            return new SwerveRequest.FieldCentric()
                    .withVelocityX(0.0)
                    .withVelocityY(0.0)
                    .withRotationalRate(rotRef[0]);
        };

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