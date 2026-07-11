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
import frc.robot.subsystems.vision.LimelightHelpers;



public class turret extends SubsystemBase {
    private TalonFX turretspinMotor = new TalonFX(canIDs.turretspinCANID, "rio");

      public turretspinState state = states.turretspinState.IDLE;

      private static Limelight shooterLL = new Limelight("limelight-shooter", 1, 0, 0, 0, false);

      private double kP = 0.008;
      private double kD = 0.003;
      private double m_lastError = 0.0;
      private double m_goalX = 1;

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
        boolean hasTarget = shooterLL.hasValidTarget();
        if(hasTarget){
          double tx = shooterLL.getTX();
          double error = m_goalX - tx;
          double dError = error - m_lastError;

          double power = kP * error + kD * dError;

          this.setSpeed(power);
          m_lastError = error;
        }else{
          this.setSpeed(0.0);
          m_lastError = 0.0;
        }
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