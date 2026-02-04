package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class SimpleTurretAim {
  private SimpleTurretAim() {
  }

  public static record AimSolution(
      Rotation2d turretYaw,
      double turretFF,
      double shooterRpmAdjust,
      double distance) {
  }

  public static AimSolution solve(
      Pose2d robotPose,
      Translation2d target,
      ChassisSpeeds robotVel,
      double leadFactor,
      double speedFactor) {

    Translation2d velocityVector = new Translation2d(robotVel.vxMetersPerSecond, robotVel.vyMetersPerSecond);

    Translation2d offset = velocityVector.times(leadFactor);
    Translation2d virtualTarget = target.minus(offset);

    Translation2d diff = virtualTarget.minus(robotPose.getTranslation());

    double distance = diff.getNorm();

    Rotation2d fieldAngle = diff.getAngle();
    Rotation2d turretYaw = fieldAngle.minus(robotPose.getRotation());

    double turretFF = 0;
    if (distance > 0.1) {
      Rotation2d robotVelAngle = velocityVector.getAngle();
      Rotation2d angleDiff = robotVelAngle.minus(fieldAngle);

      double tangentialVel = velocityVector.getNorm() * angleDiff.getSin();

      turretFF = tangentialVel / distance;
    }

    Translation2d robotToTarget = target.minus(robotPose.getTranslation());

    double dotProduct = (velocityVector.getX() * robotToTarget.getX()
        + velocityVector.getY() * robotToTarget.getY());

    double radialSpeed = 0;
    double realDist = robotToTarget.getNorm();
    if (realDist > 0.1) {
      radialSpeed = dotProduct / realDist;
    }

    double rpmAdjust = -radialSpeed * speedFactor;

    return new AimSolution(turretYaw, turretFF, rpmAdjust, distance);
  }
}