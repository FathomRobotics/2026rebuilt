// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.canIDs;
import frc.robot.subsystems.states.intakeState;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.states;

public class intake extends SubsystemBase {

  private TalonFX intakeMotor = new TalonFX(canIDs.intakeCANID, "rio");

  public intakeState state = states.intakeState.IDLE;
    
  /** Creates a new shooter. */
  public intake() {
    TalonFXConfiguration conf = new TalonFXConfiguration();

    intakeMotor.setNeutralMode(NeutralModeValue.Brake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (state) {
      case IDLE:
        this.setSpeed(0);
        break;
      case INTAKING:
        this.setSpeed(0.75);
        break;
        
    }
  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);

  }

  public void setState(intakeState newState){
    this.state = newState;
  }
    
  public Command setStateCommand(intakeState newState){
    return runOnce( () -> setState(newState));
  }

}
