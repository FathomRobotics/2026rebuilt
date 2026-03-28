// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.canIDs;
import frc.robot.subsystems.states.shooterState;
import frc.robot.subsystems.vision.Limelight;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.states;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import static edu.wpi.first.units.Units.RPM;
import static java.lang.Math.*;

public class shooter extends SubsystemBase {

  private TalonFX shooterflywheelA = new TalonFX(canIDs.shooterFlywheelACANID, "rio");
  private TalonFX shooterflywheelB = new TalonFX(canIDs.shooterFlywheelBCANID, "rio");
  private TalonFX turretKicker = new TalonFX(canIDs.turretkickerCANID, "rio");

  private static Limelight shooterLL = new Limelight("limelight-shooter", 1, 0, 0, 0, false);

  private VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);


  public shooterState state = states.shooterState.IDLE;
    
  double g = 9.8337;
  double theta = 27.25;
  double VeloLossOfMotor = 0.85;

  /** Creates a new shooter. */
  public shooter() {
    TalonFXConfiguration conf = new TalonFXConfiguration();

    shooterflywheelA.setNeutralMode(NeutralModeValue.Brake);
    shooterflywheelB.setNeutralMode(NeutralModeValue.Brake);
    turretKicker.setNeutralMode(NeutralModeValue.Brake);


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
        boolean hasTarget = shooterLL.hasValidTarget();
        if(hasTarget){
          double R = shooterLL.getTY();
          double targetVeloBall = sqrt( (R*sin(2*theta)) / g);

          double RPS = (targetVeloBall / (PI * 0.0762)) / (5/3);

          shooterflywheelA.setControl(velocityRequest.withVelocity(RPS * VeloLossOfMotor));
          shooterflywheelB.setControl(velocityRequest.withVelocity(RPS * VeloLossOfMotor));

        }else{
          this.setSpeed(0.0);
        }
        
    }
  }

  public void setSpeed(double speed) {
    shooterflywheelA.set(speed);
    shooterflywheelB.set(speed);
    turretKicker.set(speed);

  }

  public void setState(shooterState newState){
    this.state = newState;
  }
    
  public Command setStateCommand(shooterState newState){
    return runOnce( () -> setState(newState));
  }

}
