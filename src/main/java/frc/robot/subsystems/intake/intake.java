// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.canIDs;
import frc.robot.subsystems.states;
import frc.robot.subsystems.states.intakeState;

public class intake extends SubsystemBase {

  private TalonFX intakeExtendMotor = new TalonFX(canIDs.intakeExtendCANID, "rio");
  private TalonFX intakeMotor = new TalonFX(canIDs.intakeCANID, "rio");

  private double maxExtensionPoint = 15.0;

  public intakeState state = states.intakeState.IDLE;

  public intake() {
    TalonFXConfiguration conf = new TalonFXConfiguration();

    intakeExtendMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
  }
}
