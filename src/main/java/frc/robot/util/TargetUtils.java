package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.FieldConstants;

public final class TargetUtils {

  private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  static {
    // Distance (m) -> Time of Flight (sec)

    // EXAMPLE VALUES - TO BE TUNED LATER
    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);

  }

  // Position of Turret center relative to Robot Center
  private static final Translation2d robotToTurret = new Translation2d(0.0, 0.0);

  // Delay to compensate for system lag (Sensor -> Code -> Motor)
  // 0.02 - 0.04 seconds is typical.
  private static final double kPhaseDelaySec = 0.03;

  private TargetUtils() {
  }

  public static record AimSolution(
      Rotation2d turretYaw, // Turret Field-Relative Yaw Angle (Rad)
      double turretFF, // Turret Velocity Feedforward (Rad/s)
      double effectiveDistance, // Distance for RPM/Hood lookup tables
      boolean isSolutionValid // False if target is out of range
  ) {
  }

  public static Pose2d solveTargetPose(Pose2d robotPose, boolean isRed) {
    if (!isRed) {
      if (robotPose.getX() > 4.6) {
        if (robotPose.getY() > 4.0) {
          return FieldConstants.kFeedPoseBlueLeft;
        } else {
          return FieldConstants.kFeedPoseBlueRight;
        }
      } else {
        return FieldConstants.kHubPoseBlue;
      }
    } else {
      if (robotPose.getX() < 12.0) {
        if (robotPose.getY() < 4.0) {
          return FieldConstants.kFeedPoseRedLeft;
        } else {
          return FieldConstants.kFeedPoseRedRight;
        }
      } else {
        return FieldConstants.kHubPoseRed;
      }
    }
  }

  /**
   * Solves for the perfect aim angle using an iterative approach.
   * 
   * @param robotPose Current Robot Pose
   * @param robotVel  FIELD-RELATIVE ChassisSpeeds (m/s)
   * @param target    Field Target Location
   * @return AimSolution
   */
  public static AimSolution solve(
      Pose2d robotPose,
      Translation2d target,
      ChassisSpeeds robotVel) {
    // 1. Compensate for Phase Delay (Predict where robot will be in ~30ms)
    Pose2d predictedPose = robotPose.exp(
        new Twist2d(
            robotVel.vxMetersPerSecond * kPhaseDelaySec,
            robotVel.vyMetersPerSecond * kPhaseDelaySec,
            robotVel.omegaRadiansPerSecond * kPhaseDelaySec));

    // 2. Calculate Turret's Velocity in Field Space
    double robotHeadingRad = predictedPose.getRotation().getRadians();

    // V_turret = V_robot + (Omega x R_turret)
    double turretVelX = robotVel.vxMetersPerSecond
        + robotVel.omegaRadiansPerSecond * (robotToTurret.getY() * Math.cos(robotHeadingRad)
            - robotToTurret.getX() * Math.sin(robotHeadingRad));

    double turretVelY = robotVel.vyMetersPerSecond
        + robotVel.omegaRadiansPerSecond * (robotToTurret.getX() * Math.cos(robotHeadingRad)
            - robotToTurret.getY() * Math.sin(robotHeadingRad));

    // 3. Iterative Solver
    // We don't know the flight time until we know the distance.
    // We don't know the "effective distance" (aim point) until we know the flight
    // time.
    // So we loop 5 times to converge on the answer.

    // Start with the actual turret position
    Translation2d turretPos = predictedPose.transformBy(
        new edu.wpi.first.math.geometry.Transform2d(robotToTurret, new Rotation2d())).getTranslation();

    Translation2d virtualGoalPosition = target;
    double effectiveDist = target.getDistance(turretPos);
    double timeOfFlight = 0.0;

    for (int i = 0; i < 5; i++) { // 5 iterations is plenty for FRC
      // Get Time of Flight for the current estimated distance
      timeOfFlight = timeOfFlightMap.get(effectiveDist);

      // Calculate where the ball would land if we aimed exactly at the target
      // Ball Displacement = Robot Velocity * Time
      double ballDisplacementX = turretVelX * timeOfFlight;
      double ballDisplacementY = turretVelY * timeOfFlight;

      // We want the ball to land AT the target.
      // So we must aim at a "Virtual Goal" that is the Target Position MINUS the
      // Ball's lateral drift.
      virtualGoalPosition = target.minus(new Translation2d(ballDisplacementX, ballDisplacementY));

      // Recalculate distance for the next loop iteration
      effectiveDist = virtualGoalPosition.getDistance(turretPos);
    }

    // 4. Calculate Final Angles
    Translation2d diff = virtualGoalPosition.minus(turretPos);
    Rotation2d fieldAimAngle = diff.getAngle();
    Rotation2d turretYaw = fieldAimAngle.minus(predictedPose.getRotation());

    // 5. Calculate Feedforward (Turret Velocity)
    // This keeps the turret spinning at the right speed to track the target.
    double dist = diff.getNorm();
    double turretFF = 0.0;

    if (dist > 0.01) {
      Translation2d velVector = new Translation2d(turretVelX, turretVelY);
      Rotation2d velAngle = velVector.getAngle();
      Rotation2d angleDiff = velAngle.minus(fieldAimAngle);

      // Tangential component of velocity relative to target line
      double tangentialVel = velVector.getNorm() * angleDiff.getSin();
      turretFF = tangentialVel / dist;

      turretFF -= robotVel.omegaRadiansPerSecond;
    }

    Logger.recordOutput("TargetUtils/Virtual_Target_Pose", new Pose2d(virtualGoalPosition, new Rotation2d()));

    return new AimSolution(turretYaw, turretFF, effectiveDist, true);
  }
}