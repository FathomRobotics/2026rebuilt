// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.canIDs;
import frc.robot.subsystems.states;
import frc.robot.subsystems.states.intakeState;
import frc.robot.subsystems.Encoder.ThroughBoreEncoder;


public class intake extends SubsystemBase {

  //Cool Motor Stuff
  private TalonFX intakeExtendMotor = new TalonFX(canIDs.intakeExtendCANID, CANBus.roboRIO());
  private TalonFX intakeMotor = new TalonFX(canIDs.intakeCANID, CANBus.roboRIO());

  //Endoder defining
  public ThroughBoreEncoder BORRIS = new ThroughBoreEncoder(0);
  private double extensionSetpoint = states.intakeState.RETRACTING.getIntakePose(); 

  //PID
  private static final double kP = 2.5; //7
  private static final double kI = 0;
  private static final double kD = 0; //0.5
  private final PIDController intakePID = new PIDController(kP, kI, kD);
  


  private DoubleSupplier ds;

  public intakeState state = intakeState.RETRACTING;

  private boolean override = false;

  public intake() {
    BORRIS.doReset();
    intakePID.enableContinuousInput(0, 360);
    intakeExtendMotor.setNeutralMode(NeutralModeValue.Brake);
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    
  }

  public void periodic() {
        SmartDashboard.putNumber("BORRIS Encoder value", BORRIS.getDistance());
        SmartDashboard.putNumber("Target Value", extensionSetpoint);
        double output = intakePID.calculate(BORRIS.getDistance(), extensionSetpoint);

        // Clamp output
        //output = Math.max(-2, Math.min(2, output));

        //if(state == intakeState.RETRACTING){
        //intakeExtendMotor.set(output);
        SmartDashboard.putNumber("Output to Extend", output);
        //} else if(state == intakeState.EXTENDING){
        intakeExtendMotor.set(-output);
        //SmartDashboard.putNumber("Output to Extend", -output);
        //}
        

  }

  public void setExtensionSetpoint(double extensionSetpoint) {
 
        this.extensionSetpoint = extensionSetpoint;
    
  }

  public Command extendToCommand(double state) {
        intakePID.setTolerance(0.1);
        return runOnce(() -> setExtensionSetpoint(state));
    }

  public double getPostition() {
    return intakeExtendMotor.getPosition().getValueAsDouble();
  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setExtensionMotorSpeed(double speed) {
    intakeExtendMotor.set(speed);
  }

  public void setIntakeMotorSpeed(double speed) {
    intakeMotor.set(speed);
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

  public Command setSpeedCommand(double speed){
    return Commands.run( () -> setIntakeMotorSpeed(speed));
  }

  public Command setExtensionMotorSpeedCommand(double speed){
    return Commands.run( () -> setExtensionMotorSpeed(speed));
  }
}
