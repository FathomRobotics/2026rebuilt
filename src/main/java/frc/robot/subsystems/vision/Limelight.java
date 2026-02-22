// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class Limelight extends SubsystemBase {

  public static final double hubHeightMeters = 1.123;

  public static final double hubOffsetFromCenterOfTag = 0;

  public static final int[] hubIDsRed = {2, 3, 4, 5, 8, 9, 10, 11};
  public static final int[] hubIDsBlue = {18, 19, 20, 21, 24, 25, 26, 27};
  public static final int[] towerIDsRed = {15, 16};
  public static final int[] towerIDsBlue = {31, 32};

  public static final double TARGET_DEBOUNCE_TIME = 0.2;

  private int tagCount;
  public String cameraName;
  private int[] validIDs = {};
  private double tx;
  private double ty;
  private Debouncer targetDebouncer = new Debouncer(TARGET_DEBOUNCE_TIME, DebounceType.kFalling);

  public static final double angleVeloTolerance = 360 * Math.PI / 180;

  private double cameraHeightMeters;
  public double cameraAngle;
  public double cameraOffsetX;
  public double cameraOffsetY;
  private double angleMult;

  private boolean hasTipped;

  private DoublePublisher xDistPub;
  private DoublePublisher yDistPub;
  private DoublePublisher horizontalDistPub;

  public Limelight(String cameraName, double cameraHeightMeters, double cameraAngle, double cameraOffsetX, double cameraOffsetY, boolean cameraUpsdieDown) {
    this.cameraName = cameraName;
    this.cameraHeightMeters = cameraHeightMeters;
    this.cameraAngle = cameraAngle;
    this.cameraOffsetX = cameraOffsetX;
    this.cameraOffsetY = cameraOffsetY;
    LimelightHelpers.SetFiducialIDFiltersOverride(cameraName, validIDs);

    if (cameraUpsdieDown){
      angleMult = -1;
    } else {
      angleMult = 1;
    }

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable llTable = inst.getTable(cameraName);

    yDistPub = llTable.getDoubleTopic("Y Distance").publish();
    xDistPub = llTable.getDoubleTopic("X Distance").publish();
    horizontalDistPub = llTable.getDoubleTopic("Horizontal Distance").publish();
  }

  public double getTimestampSeconds() {
    double latency = (LimelightHelpers.getLimelightNTDouble(cameraName, "cl")
        + LimelightHelpers.getLimelightNTDouble(cameraName, "tl")) / 1000;

        return Timer.getFPGATimestamp() - latency;
  }

  public boolean hasValidTarget() {
    boolean hasMatch = (LimelightHelpers.getLimelightNTDouble(cameraName, "tv") == 1.0);
    return targetDebouncer.calculate(hasMatch);
  }

  public void setGyroMode(int mode) {
    LimelightHelpers.SetIMUMode(cameraName, mode);
  }

  public RawFiducial getClosestTag() {
    RawFiducial[] tags = LimelightHelpers.getRawFiducials(cameraName);
    if (tags.length == 0) {
      return null;
    }
    RawFiducial largest = tags[0];
    for (RawFiducial tag : tags) {
      if (tag.distToRobot > largest.distToRobot) {
        largest = tag;
      }
    }
    return largest;
  }

  public Rotation2d getClosestTagAngle() {
    int closestID = getClosestTag().id;
    return tagRotaionsMap.get(closestID);
  }

}
