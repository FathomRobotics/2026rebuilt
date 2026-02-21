// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {

  private final String tableName;

  private boolean hasTarget = false;
  private double tx = 0.0;
  private double ty = 0.0;

  public Limelight() {
    this("limelight");
  }

  public Limelight(String tableName) {
    this.tableName = tableName;
  }

  @Override
  public void periodic() {
    var table = NetworkTableInstance.getDefault().getTable(tableName);
    double tv = table.getEntry("tv").getDouble(0.0);
    this.hasTarget = tv >= 1.0;
    this.tx = table.getEntry("tx").getDouble(0.0);
    this.ty = table.getEntry("ty").getDouble(0.0);
  }

  public boolean hasTarget() {
    return hasTarget;
  }

  public double getX() {
    return tx;
  }

  public double getY() {
    return ty;
  }

  public void setPipeline(int pipeline) {
    NetworkTableInstance.getDefault().getTable(tableName).getEntry("pipeline").setNumber(pipeline);
  }
  
  public void setLedMode(int mode) {
    NetworkTableInstance.getDefault().getTable(tableName).getEntry("ledMode").setNumber(mode);
  }
}
