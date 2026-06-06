// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.canIDs;
import frc.robot.subsystems.states;
import frc.robot.subsystems.states.intakeState;

public class intake extends SubsystemBase {

  private TalonFX intakeExtendMotor1 = new TalonFX(canIDs.intakeExtend1CANID, CANBus.roboRIO());
  private TalonFX intakeExtendMotor2 = new TalonFX(21, CANBus.roboRIO());
  private TalonFX intakeMotor = new TalonFX(canIDs.intakeCANID, CANBus.roboRIO());

  private double maxExtension = 15;
  private double rotationsPerSec = maxExtension / 30;

  private double targetPose1 = 0;
  private final MotionMagicVoltage motionControl = new MotionMagicVoltage(0);

  private DoubleSupplier ds;

  public intakeState state = states.intakeState.IDLE;

  private boolean override = false;

  public intake() {
    TalonFXConfiguration conf = new TalonFXConfiguration();
    MotorOutputConfigs motorConf = new MotorOutputConfigs();

    intakeExtendMotor1.setNeutralMode(NeutralModeValue.Brake);
    intakeExtendMotor2.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);

    motorConf.withDutyCycleNeutralDeadband(.1);
    motorConf.withNeutralMode(NeutralModeValue.Brake);

    var limitConf = new CurrentLimitsConfigs();

    limitConf.StatorCurrentLimit = 40;
    limitConf.StatorCurrentLimitEnable = true;

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
      StatusCode status4 = StatusCode.StatusCodeNotInitialized; 

      for (int i = 0; i < 5; ++i) {
        status = intakeExtendMotor1.getConfigurator().apply(conf);
        status2 = intakeMotor.getConfigurator().apply(conf);
        status3 = intakeExtendMotor2.getConfigurator().apply(motorConf);
        status4 = intakeExtendMotor2.getConfigurator().apply(conf);
        intakeExtendMotor1.getConfigurator().apply(limitConf);
        intakeExtendMotor2.getConfigurator().apply(limitConf);
        if (status.isOK() && status2.isOK() && status3.isOK() && status4.isOK()) break;
      }
      if (!status.isOK()) {
        System.out.println("Could not configure device. Error: " + status.toString());
      }
  }

  @Override
  public void periodic() {
    intakeExtendMotor2.setControl(motionControl.withPosition(targetPose1).withSlot(0));
   intakeExtendMotor1.setControl(new Follower(intakeExtendMotor2.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public void goToPose(double newPosition) {
    this.override = false;
    this.targetPose1 = newPosition;
  }

  public boolean getAtPose(){
    return Math.abs(this.intakeExtendMotor1.getPosition().getValueAsDouble() - this.targetPose1) < 0.05;
  }
  public double getPostition() {
    return intakeExtendMotor1.getPosition().getValueAsDouble();
  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setExtensionMotorSpeed(double speed) {
    intakeExtendMotor1.set(speed);
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

  public Command setExtensionMotorSpeedCommand(DoubleSupplier speed){
    return Commands.run( () -> setExtensionMotorSpeed(speed.getAsDouble()));
  }
}
