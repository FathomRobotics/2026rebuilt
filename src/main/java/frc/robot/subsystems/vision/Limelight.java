// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.LimelightHelpers.RawFiducial;

import static edu.wpi.first.units.Units.Rotation;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class Limelight extends SubsystemBase {

  public static final double hubHeightMeters = 1.123;
  public static final double towerHeightMeters = 0.55245;

  public static final double hubOffsetFromCenterOfTag = 0;

  public static final int[] hubIDsRed = {2, 3, 4, 5, 8, 9, 10, 11};
  public static final int[] hubIDsBlue = {18, 19, 20, 21, 24, 25, 26, 27};
  public static final int[] hubIDs = {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27};
  public static final int[] towerIDsRed = {15, 16};
  public static final int[] towerIDsBlue = {31, 32};
  public static HashMap<Integer, Rotation2d> tagRotationsMap = new HashMap<Integer, Rotation2d>();
  {
    tagRotationsMap.put(2, Rotation2d.fromDegrees(90));
    tagRotationsMap.put(3, Rotation2d.fromDegrees(180));
    tagRotationsMap.put(4, Rotation2d.fromDegrees(180));
    tagRotationsMap.put(5, Rotation2d.fromDegrees(270));
    tagRotationsMap.put(8, Rotation2d.fromDegrees(270));
    tagRotationsMap.put(9, Rotation2d.fromDegrees(0));
    tagRotationsMap.put(10, Rotation2d.fromDegrees(0));
    tagRotationsMap.put(11, Rotation2d.fromDegrees(90));

    tagRotationsMap.put(18, Rotation2d.fromDegrees(90));
    tagRotationsMap.put(19, Rotation2d.fromDegrees(180));
    tagRotationsMap.put(20, Rotation2d.fromDegrees(180));
    tagRotationsMap.put(21, Rotation2d.fromDegrees(270));
    tagRotationsMap.put(24, Rotation2d.fromDegrees(270));
    tagRotationsMap.put(25, Rotation2d.fromDegrees(0));
    tagRotationsMap.put(26, Rotation2d.fromDegrees(0));
    tagRotationsMap.put(27, Rotation2d.fromDegrees(90));
  }

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

  public static boolean isCorrectID(int ID, int... IDs) {
    for (int n : IDs) {
      if (n == ID) 
        return true;
    }
    return false;
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
    return tagRotationsMap.get(closestID);
  }

  public double getDistanceToTag(double tagHeightMeters) {
    if (hasValidTarget()) {
      double distance = getStraightDistanceToTag(tagHeightMeters) - cameraOffsetY;
      return distance / Math.cos((Math.PI / 180.0) * getTX());
    }
    return 0;
  }

  public double getStraightDistanceToTag(double tagHeightMeters) {
    if (hasValidTarget()) {
      double distance = (tagHeightMeters - cameraHeightMeters)
        / Math.tan(
          (Math.PI / 180.0)
            * (cameraAngle + getTY()));
    }
    return 0;
  }

  public double getHorizontalDistanceToTag(double tagHeightMeters) {
    if (hasValidTarget()) {
      double distance = getStraightDistanceToTag(tagHeightMeters) - cameraOffsetY;
       distance = distance * Math.tan(getTX() * (Math.PI / 180.0));
       return distance + cameraOffsetX;
    }
    return 0;
  }

  public double getDistanceToHub() {
    return getDistanceToTag(hubHeightMeters);
  }

  public double getStraightDistanceToHub() {
    return getStraightDistanceToTag(hubHeightMeters);
  }

  public double getHorizontalDistanceToHub() {
    return getHorizontalDistanceToTag(hubHeightMeters);
  }

  public double getDistanceToTower() {
    return getDistanceToTag(towerHeightMeters);
  }

  public double getStraightDistanceToTower() {
    return getStraightDistanceToTag(towerHeightMeters);
  }

  public double getHorizontalDistanceToTower() {
    return getHorizontalDistanceToTag(towerHeightMeters);
  }

  @AutoLogOutput
  public double getTX() {
    return tx * angleMult;
  }

  @AutoLogOutput
  public double getTY() {
    return ty * angleMult;
  }

  public DoubleSupplier tySupplier() {
    return () -> getTY();
  }

  public DoubleSupplier txSupplier() {
    return () -> getTX();
  }

  public double getStrafeDistanceToTower() {
    if (isCorrectID(getTagID(), hubIDs)) {
      return (Math.tan(Math.toRadians(getTX()))) * getStraightDistanceToTower();
    }
    return 0;
  }

  public int getTagID() {
    return (int) LimelightHelpers.getFiducialID(cameraName);
  }

  public void periodic() {
    tx = LimelightHelpers.getTX(cameraName);
    ty = LimelightHelpers.getTY(cameraName);
    RawFiducial[] allTags = LimelightHelpers.getRawFiducials(cameraName);
    int numValidTags = 0;
    for (LimelightHelpers.RawFiducial t : allTags) {
      if (t.distToCamera < 4.0) {
        numValidTags ++;
      }
    }

    int[] validTags = new int[numValidTags];
    int counter = 0;
    for (RawFiducial t : allTags) {
      if (t.distToCamera < 4.0) {
        validTags[counter] = t.id;
        counter++;
      }
    }
    xDistPub.set(getHorizontalDistanceToHub());
    yDistPub.set(getStraightDistanceToHub());
    horizontalDistPub.set(getDistanceToHub());

    double[] poseArr = LimelightHelpers.getBotPose_TargetSpace(cameraName);
    Pose2d botPose = new Pose2d();
    if (poseArr.length >= 6) {
      botPose = new Pose2d(poseArr[0], poseArr[2], Rotation2d.fromDegrees(poseArr[4]));
    }
    Logger.recordOutput(cameraName + "/IMUYaw",
      LimelightHelpers.getIMUData(cameraName).robotYaw * (Math.PI / 180.0));
    Logger.recordOutput(cameraName + "/BotPoseTargetSapce", botPose); 
    Logger.recordOutput(cameraName + "/BotPose3dTargetSpace",
      LimelightHelpers.getBotPose3d_TargetSpace(cameraName));

    var entry = LimelightHelpers.getLimelightNTTableEntry(cameraName, "tcornxy");
    if (entry != null) {
      var tcornxy = entry.getDoubleArray(new double[0]);
      if (tcornxy != null && tcornxy.length > 0) {
        Logger.recordOutput(cameraName + "/tcornxy", tcornxy);
      }
    }
  }

  public Command flashLEDs() {
    return Commands.sequence(
      Commands.runOnce(() -> LimelightHelpers.setLEDMode_ForceBlink(cameraName)),
      Commands.waitSeconds(0.6),
      Commands.runOnce(() -> LimelightHelpers.setLEDMode_ForceOff(cameraName))
    );
  }
  
  public Command ifHasTarget(Command cmd) {
    return cmd.onlyWhile(this::hasValidTarget);
  }
}
