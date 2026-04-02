package frc.robot.subsystems.hood;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.canIDs;
import frc.robot.subsystems.states;
import frc.robot.subsystems.states.hoodState;

public class hood extends SubsystemBase {

  private TalonFX hoodMotor = new TalonFX(canIDs.shooterHoodCANID, CANBus.roboRIO());

  private double maxExtension = 15;
  private double rotationsPerSec = maxExtension / 30;

  private double targetPose = 0;
  private final MotionMagicVoltage motionControl = new MotionMagicVoltage(0);

  private DoubleSupplier ds;

  public hoodState state = states.hoodState.DOWN;

  private boolean override = false;

  public hood() {
    TalonFXConfiguration conf = new TalonFXConfiguration();
    MotorOutputConfigs motorConf = new MotorOutputConfigs();

    hoodMotor.setNeutralMode(NeutralModeValue.Brake);

    motorConf.withDutyCycleNeutralDeadband(1);
    motorConf.withNeutralMode(NeutralModeValue.Brake);

    var limitConf = new CurrentLimitsConfigs();

    limitConf.StatorCurrentLimit = 100;
    limitConf.StatorCurrentLimitEnable = false;

    var motionMagicConfigs = conf.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 30;
    motionMagicConfigs.MotionMagicAcceleration = 30;
    motionMagicConfigs.MotionMagicJerk = 30;

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
        status = hoodMotor.getConfigurator().apply(conf);
        status3 = hoodMotor.getConfigurator().apply(motorConf);
        hoodMotor.getConfigurator().apply(limitConf);
        if (status.isOK() && status3.isOK()) break;
      }
      if (!status.isOK()) {
        System.out.println("Could not configure device. Error: " + status.toString());
      }
  }

  @Override
  public void periodic() {
    hoodMotor.setControl(motionControl.withPosition(targetPose).withSlot(0));
  }

  public void goToPose(double newPosition) {
    this.override = false;
    this.targetPose = newPosition;
  }

  public boolean getAtPose(){
    return Math.abs(this.hoodMotor.getPosition().getValueAsDouble() - this.targetPose) < 0.05;
  }
  public double getPostition() {
    return hoodMotor.getPosition().getValueAsDouble();
  }

  public void setHoodMotorSpeed(double speed) {
    hoodMotor.set(speed);
  }
  public Command goToPositionCommand(double target){
    return Commands.runOnce( ()-> goToPose(target));
  }


  public void enableOverride(DoubleSupplier power){
    this.ds = power;
    this.override = true;
  }

  public void setState(hoodState newState){
    this.state = newState;
  }
    
  public Command setStateCommand(hoodState newState){
    return runOnce( () -> setState(newState));
  }

  public Command setHoodMotorSpeedCommand(DoubleSupplier speed){
    return Commands.run( () -> setHoodMotorSpeed(speed.getAsDouble()));
  }
}