// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.canIDs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.states;
import frc.robot.subsystems.states.spindexState;
import frc.robot.subsystems.states.turretspinState;
import frc.robot.subsystems.vision.Limelight;



public class turret extends SubsystemBase {
    private TalonFX turretspinMotor = new TalonFX(canIDs.turretspinCANID, "rio");

      public turretspinState state = states.turretspinState.IDLE;

      private static Limelight shooterLL = new Limelight("limelight-objtrac", 1, 0, 0, 0, false);

      double tx = shooterLL.getTX();
      double headingError = -tx;
      double KpAim = -0.04;
      double minAimCommand = 0.05;

  /** Creates a new turret. */
  public turret() {
    TalonFXConfiguration conf = new TalonFXConfiguration();

    turretspinMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    switch (state) {
      case IDLE:
        this.setSpeed(0);
        break;
      case TRACKING:
        double steeringAdjust = 0.0;
        if (tx > 1.0) {
            steeringAdjust = (KpAim * headingError) - minAimCommand;
          }
        else if (tx < -1.0) {
            steeringAdjust = (KpAim * headingError) + minAimCommand;
          }
        
        this.setSpeed(steeringAdjust);
        break;
    }

  }

  public void setSpeed(double speed) {
    turretspinMotor.set(speed);
  }

  public void setState(turretspinState newState){
    this.state = newState;
  }
    
  public Command setStateCommand(turretspinState newState){
    return runOnce( () -> setState(newState));
  }
}
