package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class TurretAimMath {
  private TurretAimMath() {}

  public static record AimSolution(
      double distanceMeters,
      Rotation2d fieldToTargetAngle,
      Rotation2d turretYawRelativeToRobot
  ) {}

  // Pose will be added later on.
  private static Pose2d targetPoseRed = new Pose2d(0, 0, new Rotation2d());
  private static Pose2d targetPoseBlue = new Pose2d(0, 0, new Rotation2d());

  public static AimSolution solveForBasket(Pose2d robotPose, boolean isRed) {
    Translation2d target = isRed ? targetPoseRed.getTranslation() : targetPoseBlue.getTranslation();
    return solve(robotPose, target);
  }

  public static AimSolution solve(Pose2d robotPose, Translation2d target) {
    Translation2d robotXY = robotPose.getTranslation();
    Translation2d delta = target.minus(robotXY);

    double distance = delta.getNorm();
    Rotation2d fieldAngleToTarget = delta.getAngle();

    Rotation2d turretYaw = fieldAngleToTarget.minus(robotPose.getRotation());

    return new AimSolution(distance, fieldAngleToTarget, turretYaw);
  }

  public static Rotation2d turretYawToTarget(Pose2d robotPose, Translation2d target) {
    return solve(robotPose, target).turretYawRelativeToRobot();
  }
}