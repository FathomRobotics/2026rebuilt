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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.Generated.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.spindexer.spindexer;
import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.turret.turret;
import frc.robot.subsystems.intake.intake;
import frc.robot.subsystems.hood.hood;

import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.LimelightHelpers;
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

    // Creates a SysIdRoutine

    public final spindexer spindexer = new spindexer();
    public final shooter shooter = new shooter();
    public final turret turret = new turret();
    public final intake intake = new intake();
    public final hood hood = new hood();




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
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.cross().onTrue(
            Commands.sequence(
                shooter.setStateCommand(states.shooterState.SHOOTING),
                turret.setStateCommand(states.turretspinState.TRACKING)

                //intake.setStateCommand(states.intakeState.INTAKING)

           )
        );
        joystick.L1().onTrue(
            Commands.sequence(
                spindexer.setStateCommand(states.spindexState.IDLE),
                shooter.setStateCommand(states.shooterState.IDLE),
                turret.setStateCommand(states.turretspinState.IDLE)

                //intake.setStateCommand(states.intakeState.IDLE)


            )
        );
        joystick.R1().onTrue(
            Commands.sequence(
                spindexer.setStateCommand(states.spindexState.RUNNING)
                //intake.setStateCommand(states.intakeState.IDLE)
        

            )
        );


        //joystick2.povUp().onTrue(
        //  Commands.sequence(
        //    this.hood.goToPositionCommand(states.hoodState.UP.getHoodPose()),
        //        this.hood.setStateCommand(states.hoodState.UP)
        //  )
        //);
        //joystick2.povDown().onTrue(
        //  Commands.sequence(
        //    this.hood.goToPositionCommand(states.hoodState.DOWN.getHoodPose()),
        //        this.hood.setStateCommand(states.hoodState.DOWN)
        //  )
        //);
        
        //joystick2.L1().onTrue(
        //  Commands.sequence(
        //    this.intake.setSpeedCommand(() -> 1.0),
        //        this.intake.setStateCommand(states.intakeState.EXTENDED)
        //  )
        //);

        //joystick2.L1().onFalse(
        //    Commands.sequence(
        //        this.intake.setSpeedCommand(() -> 0),
        //        this.intake.setStateCommand(states.intakeState.EXTENDED)
        //        )
        //    );

        //joystick2.touchpad().onTrue(
        //    Commands.sequence(
        //        this.intake.setSpeedCommand(() -> -0.5))
        //);
        
        //joystick2.square().onTrue(
        //  Commands.sequence(
        //    this.intake.goToPositionCommand(states.intakeState.EXTENDING.getIntakePose()),
        //        this.intake.setStateCommand(states.intakeState.EXTENDING)
        //  )
        //);

        //joystick2.triangle().onTrue(
        //    Commands.sequence(
        //        this.intake.goToPositionCommand(states.intakeState.RETRACTING.getIntakePose()),
        //        this.intake.setStateCommand(states.intakeState.RETRACTING)
        //    )
        //);        


        //this.shooter.setSpeed(joystick2.getLeftX() * 0.5);

        //joystick.triangle().whileTrue(drivetrain.applyRequest(() -> brake));
        //joystick.square().whileTrue(drivetrain.applyRequest(() ->
        //    point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        //));

        // Reset the field-centric heading on left bumper press.
        joystick.povUp().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
             return autoChooser.selectedCommand();

    }
}
