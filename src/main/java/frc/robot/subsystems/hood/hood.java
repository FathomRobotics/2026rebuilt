package frc.robot.subsystems.hood;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.canIDs;
import frc.robot.subsystems.states.hoodState;
import frc.robot.subsystems.states.shooterState;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.states;

public class hood extends SubsystemBase {

  private TalonFX shooterHood = new TalonFX(canIDs.shooterHoodCANID, "rio");

  public hoodState state = states.hoodState.IDLE;
    
  /** Creates a new shooter. */
  public hood() {
    TalonFXConfiguration conf = new TalonFXConfiguration();

    shooterHood.setNeutralMode(NeutralModeValue.Brake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (state) {
      case IDLE:
        this.setSpeed(0);
        break;
      case TRACKING:
        this.setSpeed(1);
        break;
        
    }
  }

  public void setSpeed(double speed) {
    shooterHood.set(speed);

  }

  public void setState(hoodState newState){
    this.state = newState;
  }
    
  public Command setStateCommand(hoodState newState){
    return runOnce( () -> setState(newState));
  }

}

