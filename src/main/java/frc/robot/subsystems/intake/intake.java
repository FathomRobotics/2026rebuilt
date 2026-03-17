// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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

  private TalonFX intakeExtendMotor = new TalonFX(canIDs.intakeExtendCANID, "rio");
  private TalonFX intakeMotor = new TalonFX(canIDs.intakeCANID, "rio");

  private double maxExtension = 15;
  private double rotationsPerSec = maxExtension / 84;

  private double targetPose = 0;
  private final MotionMagicVoltage motionControl = new MotionMagicVoltage(0);

  private DoubleSupplier ds;

  public intakeState state = states.intakeState.IDLE;

  private boolean override = false;

  public intake() {
    TalonFXConfiguration conf = new TalonFXConfiguration();

    intakeExtendMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);

    var limitConf = new CurrentLimitsConfigs();

    FeedbackConfigs fbConf = new FeedbackConfigs();

    var motionMagicConfigs = conf.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 10;
    motionMagicConfigs.MotionMagicAcceleration = 10 * 2;
    motionMagicConfigs.MotionMagicJerk = 10 * 2 * 3;

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
    switch (state) {
      case IDLE:
        this.setSpeed(0);
        break;
      case INTAKING:
        this.setIntakeMotorSpeed(1);
        break;
      case EXTENDING:
        this.setIntakeMotorSpeed(0);
        this.goToPose(maxExtension);
        break;
      case EXTENDED:
        this.setIntakeMotorSpeed(1);
        break;
      case RETRACTING:
        this.setIntakeMotorSpeed(0);
        this.goToPose(0);
        break;
    }
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
    intakeMotor.set(speed);
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
}
