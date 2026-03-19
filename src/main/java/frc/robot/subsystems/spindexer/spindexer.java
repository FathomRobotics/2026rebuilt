// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.canIDs;
import frc.robot.subsystems.states;
import frc.robot.subsystems.states.spindexState;

public class spindexer extends SubsystemBase {

  private TalonFX spindexerMotor = new TalonFX(canIDs.spindexCANID, CANBus.roboRIO());
  private TalonFX kickerMotor = new TalonFX(canIDs.turretkickerCANID, CANBus.roboRIO());

  public spindexState state = states.spindexState.IDLE;

  public spindexer() {
    TalonFXConfiguration conf = new TalonFXConfiguration();

    conf.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    spindexerMotor.setNeutralMode(NeutralModeValue.Brake);
    kickerMotor.setNeutralMode(NeutralModeValue.Brake);

    spindexerMotor.getConfigurator().apply(conf);
  }

  @Override
  public void periodic() {
    
    switch (state) {
      case IDLE:
        this.setSpeed(0);
        break;
      case RUNNING:
        this.setSpeed(.75);
        break;
    }
  }

  public void setSpeed(double speed) {
    spindexerMotor.set(speed);
    kickerMotor.set(speed);
  }

  public void setState(spindexState newState){
    this.state = newState;
  }
    
  public Command setStateCommand(spindexState newState){
    return runOnce( () -> setState(newState));
  }
}
