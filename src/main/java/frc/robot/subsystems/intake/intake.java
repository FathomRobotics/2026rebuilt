// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
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
import frc.robot.subsystems.states.intakeState;

public class intake extends SubsystemBase {

  private TalonFX intakeExtendMotor = new TalonFX(canIDs.intakeExtendCANID, CANBus.roboRIO());
  private TalonFX intakeMotor = new TalonFX(canIDs.intakeCANID, CANBus.roboRIO());

  private double maxExtension = 15;
  private double rotationsPerSec = maxExtension / 30;

  private double targetPose = 0;
  private final MotionMagicVoltage motionControl = new MotionMagicVoltage(0);

  private DoubleSupplier ds;

  public intakeState state = states.intakeState.IDLE;

  private boolean override = false;

  public intake() {
    TalonFXConfiguration conf = new TalonFXConfiguration();

    // intakeExtendMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);

    var limitConf = new CurrentLimitsConfigs();

    limitConf.StatorCurrentLimit = 90;
    limitConf.StatorCurrentLimitEnable = true;

    var motionMagicConfigs = conf.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 50;
    motionMagicConfigs.MotionMagicAcceleration = 50;
    motionMagicConfigs.MotionMagicJerk = 10 * 2 * 3;

    Slot0Configs slot0 = conf.Slot0;
    slot0.kP = 0.5;
    slot0.kV = 4;
    slot0.kA = 0.1;
    slot0.kP = 25;
    slot0.kI = 0;
    slot0.kD = 0;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
      StatusCode status2 = StatusCode.StatusCodeNotInitialized; 
      for (int i = 0; i < 5; ++i) {
        status = intakeExtendMotor.getConfigurator().apply(conf);
        status2 = intakeMotor.getConfigurator().apply(conf); 
        intakeExtendMotor.getConfigurator().apply(limitConf);
        if (status.isOK() && status2.isOK()) break;
      }
      if (!status.isOK()) {
        System.out.println("Could not configure device. Error: " + status.toString());
      }
  }

  @Override
  public void periodic() {
    intakeExtendMotor.setControl(motionControl.withPosition(targetPose).withSlot(0));
  }

  public void goToPose(double newPosition) {
    this.override = false;
    this.targetPose = newPosition;
  }

  public boolean getAtPose(){
    return Math.abs(this.intakeExtendMotor.getPosition().getValueAsDouble() - this.targetPose) < 0.05;
  }
  public double getPostition() {
    return intakeExtendMotor.getPosition().getValueAsDouble();
  }

  public void setSpeed(double speed) {
    // intakeMotor.set(speed);
    intakeExtendMotor.set(speed);
  }

  public void setIntakeMotorSpeed(double speed) {
    intakeMotor.set(speed);
  }


  public Command goToPositionCommand(double target){
    return Commands.runOnce( ()-> goToPose(target));
  }


  public void enableOverride(DoubleSupplier power){
    this.ds = power;
    this.override = true;
  }

  public void setState(intakeState newState){
    this.state = newState;
  }
    
  public Command setStateCommand(intakeState newState){
    return runOnce( () -> setState(newState));
  }

  public Command setSpeedCommand(DoubleSupplier speed){
    return Commands.run( () -> setSpeed(speed.getAsDouble()));
  }
}
