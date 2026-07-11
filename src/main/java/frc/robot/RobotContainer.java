// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;


import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.Generated.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.spindexer.spindexer;
import frc.robot.subsystems.turret.turret;
import frc.robot.subsystems.intake.intake;
import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.states;

public class RobotContainer {

    private final AutoFactory autoFactory;
    private final AutoRoutines autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public final spindexer spindexer = new spindexer();
    public final intake intake = new intake();
    public final shooter shooter = new shooter();
    public final turret turret = new turret();


    private static Limelight shooterLL = new Limelight("limelight-shooter", 1, 0, 0, 0, false);
    private static Limelight intakeLL = new Limelight("limelight-intake", 1, 0, 0, 0, false);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS5Controller joystick = new CommandPS5Controller(0);
    private final CommandPS5Controller joystick2 = new CommandPS5Controller(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();

        autoFactory = drivetrain.createAutoFactory();

        autoRoutines = new AutoRoutines(autoFactory,this.turret, this.shooter, this.spindexer, this.intake, this.shooterLL);
        autoChooser.addRoutine("Right Auto", autoRoutines::RightAuto);
        autoChooser.addRoutine("Center Auto", autoRoutines::CenterAuto);
        autoChooser.addRoutine("Center Auto V2", autoRoutines::CenterAutoVer2);
        autoChooser.addRoutine("Left Auto", autoRoutines::LeftAuto);



        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // joystick.circle().onTrue(new drivetrainaprtagtrack(drivetrain, aprTagLL));
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();

        this.intake.setState(states.intakeState.IDLE);
        this.spindexer.setState(states.spindexState.IDLE);
        this.shooter.setState(states.shooterState.IDLE);

        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        joystick.cross().onTrue(
            Commands.sequence(
                shooter.setStateCommand(states.shooterState.SHOOTING),
                turret.setStateCommand(states.turretspinState.TRACKING)
           )
        );
        joystick.L1().onTrue(
            Commands.sequence(
                spindexer.setStateCommand(states.spindexState.IDLE),
                shooter.setStateCommand(states.shooterState.IDLE),
                turret.setStateCommand(states.turretspinState.IDLE)

            )
        );
        joystick.R1().onTrue(
            Commands.sequence(
                spindexer.setStateCommand(states.spindexState.RUNNING)

            )
        );

        joystick2.triangle().onTrue(
            this.intake.extendToCommand(states.intakeState.RETRACTING.getIntakePose())
        );
        joystick2.triangle().onTrue(
            this.intake.setStateCommand(states.intakeState.RETRACTING)
        );
        joystick2.square().onTrue(
            this.intake.extendToCommand(states.intakeState.EXTENDING.getIntakePose())
        );
        joystick2.square().onTrue(
            this.intake.setStateCommand(states.intakeState.EXTENDING)
        );
        joystick2.cross().whileTrue(
            this.intake.setSpeedCommand(0.5)
        );
        joystick2.cross().whileFalse(
            this.intake.setSpeedCommand(0)
        );
        joystick2.circle().onTrue(
            Commands.sequence(
            this.intake.extendToCommand(states.intakeState.AGITATE.getIntakePose()),
            this.intake.setStateCommand(states.intakeState.AGITATE),
            new WaitCommand(0.7),
            this.intake.extendToCommand(states.intakeState.EXTENDING.getIntakePose()),
            this.intake.setStateCommand(states.intakeState.EXTENDING),
            new WaitCommand(0.7),
            this.intake.extendToCommand(states.intakeState.AGITATE.getIntakePose()),
            this.intake.setStateCommand(states.intakeState.AGITATE),
            new WaitCommand(0.7),
            this.intake.extendToCommand(states.intakeState.EXTENDING.getIntakePose()),
            this.intake.setStateCommand(states.intakeState.EXTENDING)
            )
        );
        

        // joystick.cross().onFalse(
        //     Commands.sequence(
        //         this.spindexer.setStateCommand(states.spindexState.IDLE),
        //         this.intake.setExtensionMotorSpeedCommand(() -> 0)
        //     )
        // );

        // joystick.R1().onTrue(
        //     if (this.intake.intakeEncoder.get() > 1) {
        //        this.intake.setExtensionMotorSpeedCommand(() -> 0.2);
        //     } else {
        //         this.intake.setExtensionMotorSpeedCommand(() -> 0);
        //     }
        // );
 
        // joystick.triangle().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.square().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));
 
        // Reset the field-centric heading on left bumper press.
        joystick.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric(drivetrain .getPigeon2().getRotation2d())));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {

        return autoChooser.selectedCommand();


    }
}
