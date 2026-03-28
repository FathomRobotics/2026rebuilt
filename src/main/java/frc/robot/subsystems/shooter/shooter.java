// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.canIDs;
import frc.robot.subsystems.states.shooterState;
import frc.robot.subsystems.vision.Limelight;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.states;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import static edu.wpi.first.units.Units.RPM;
import static java.lang.Math.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

public class shooter extends SubsystemBase {

  private TalonFX shooterflywheelA = new TalonFX(canIDs.shooterFlywheelACANID, "rio");
  private TalonFX shooterflywheelB = new TalonFX(canIDs.shooterFlywheelBCANID, "rio");

  private static Limelight shooterLL = new Limelight("limelight-shooter", 1, 0, 0, 0, false);

  public shooterState state = states.shooterState.IDLE;

  // Control request object for velocity control
  private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);

  // Falcon 500 has 2048 encoder ticks per revolution
  private static final double TICKS_PER_REV = 2048.0;

  private final InterpolatingDoubleTreeMap m_table = new InterpolatingDoubleTreeMap();
    
  /** Creates a new shooter. */
  public shooter() {
    TalonFXConfiguration conf = new TalonFXConfiguration();

    Slot0Configs slot0 = conf.Slot0;
    slot0.kV = 4;
    slot0.kA = 0.1;
    slot0.kP = 5;
    slot0.kI = 0;
    slot0.kD = 0;
    slot0.kS = 1;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
      StatusCode status2 = StatusCode.StatusCodeNotInitialized; 
      StatusCode status3 = StatusCode.StatusCodeNotInitialized; 

      for (int i = 0; i < 5; ++i) {
        status = shooterflywheelA.getConfigurator().apply(conf);
        status2 = shooterflywheelB.getConfigurator().apply(conf);
        if (status.isOK() && status2.isOK() && status3.isOK()) break;
      }
      if (!status.isOK()) {
        System.out.println("Could not configure device. Error: " + status.toString());
      }

    shooterflywheelA.setNeutralMode(NeutralModeValue.Brake);
    shooterflywheelB.setNeutralMode(NeutralModeValue.Brake);


    // Add data points: put(key, value)
    m_table.put(0.0, 1000.0); // At 0 meters, 1000 RPM
    m_table.put(0.0, 1000.0); // At 0 meters, 1000 RPM
    m_table.put(0.0, 1000.0); // At 0 meters, 1000 RPM
    m_table.put(0.0, 1000.0); // At 0 meters, 1000 RPM

    SmartDashboard.putNumber("Shooter Shooting Target RPM", 1000.0);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (state) {
      case IDLE:
        this.setSpeed(0);
        break;
      case SPINNING_UP:
        this.setSpeed(1);
        break;
      case READY_TO_SHOOT:
        this.setSpeed(1);
        break;
      case SHOOTING:
        // Example: run motor at ____ RPM
        double targetRPM = SmartDashboard.getNumber("Shooter Shooting Target RPM", 1000.0);

        // Convert RPM to rotations per second
        double targetRPS = targetRPM / 60.0;

        // Command the motor in RPS (Phoenix 6 uses rotations/sec for velocity)
        shooterflywheelA.setControl(velocityRequest.withVelocity(targetRPS));
        shooterflywheelB.setControl(velocityRequest.withVelocity(targetRPS));
        break;
        
    }
  }

  public void setSpeed(double speed) {
    shooterflywheelA.set(speed);
    shooterflywheelB.set(speed);

  }

  public void setState(shooterState newState){
    this.state = newState;
  }
    
  public Command setStateCommand(shooterState newState){
    return runOnce( () -> setState(newState));
  }

}