//will do later

// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import com.ctre.phoenix6.swerve.SwerveRequest;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
// import frc.robot.subsystems.vision.Limelight;

// public class fuelautoallign extends Command {
//   private final CommandSwerveDrivetrain drivetrain;
//   private final Limelight limelight;
//   private final PIDController turnController;

//   private final double maxAngularSpeed = 3.0; 

//   public fuelautoallign(CommandSwerveDrivetrain drivetrain, Limelight limelight) {
//     this.drivetrain = drivetrain;
//     this.limelight = limelight;

//   this.turnController = new PIDController(0.02, 0.0, 0.0);
//   turnController.setTolerance(1.5);
//   }

//   @Override
//   public void initialize() {
//     turnController.reset();
//   }

//   @Override
//   public void execute() {
//     if (!limelight.hasTarget()) {
//       // no target: stop rotation
//       drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0, 0, 0)));
//       return;
//     }

//     double errorDeg = limelight.getX();
//     double output = turnController.calculate(errorDeg, 0.0);

//     double omega = Math.toRadians(output);

//     // clamp
//     if (omega > maxAngularSpeed) {
//       omega = maxAngularSpeed;
//     } else if (omega < -maxAngularSpeed) {
//       omega = -maxAngularSpeed;
//     }

//     drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0.0, 0.0, omega)));
//   }

//   @Override
//   public void end(boolean interrupted) {
//     drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0, 0, 0)));
//   }

//   @Override
//   public boolean isFinished() {
//     return limelight.hasTarget() && turnController.atSetpoint();
//   }
// }
