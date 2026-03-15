// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.canIDs;

public class spindexer extends SubsystemBase {

  private TalonFX spindexerMotor = new TalonFX(canIDs.spindexCANID, "rio");

  public spindexer() {
    TalonFXConfiguration conf = new TalonFXConfiguration();

    spindexerMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
  }
}
