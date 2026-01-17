package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.util.TurretAimMath;

public class VisionSubsystem extends SubsystemBase {

    public enum CameraID {
        RIGHT("limelight-right"),
        LEFT("limelight-left"),
        BACK("limelight-back");

        public final String name;

        CameraID(String name) {
            this.name = name;
        }
    }

    private final SwerveDrivePoseEstimator m_poseEstimatorConsumer;

    private double m_stdDevX = 0.6; // meters
    private double m_stdDevY = 0.6; // meters
    private double m_stdDevTheta = 9999999.0;

    // Minimum quality thresholds for accepting MT2 estimates
    private int m_minTagCount = 1;
    private double m_maxAmbiguity = 0.5; // ignore very ambiguous targets
    private double m_maxDistanceMeters = 6.0; // ignore super-far tags

    private double m_angularVelocityDegPerSec = 0.0;
    private double m_linearVelocityMps = 0.0;

    public VisionSubsystem(SwerveDrivePoseEstimator poseEstimatorConsumer) {
        m_poseEstimatorConsumer = poseEstimatorConsumer;
        m_poseEstimatorConsumer.setVisionMeasurementStdDevs(
                VecBuilder.fill(m_stdDevX, m_stdDevY, m_stdDevTheta));
    }

    // ------------------------
    // External motion inputs
    // ------------------------

    public void setRobotMotion(double linearVelocityMps, double angularVelocityDegPerSec) {
        m_linearVelocityMps = linearVelocityMps;
        m_angularVelocityDegPerSec = angularVelocityDegPerSec;
    }

    // ------------------------
    // Public query helpers
    // ------------------------

    public Pose2d getLatestPose(CameraID camera) {
        LimelightHelpers.PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera.name);
        if (!isEstimateUsable(est)) {
            return null;
        }
        return est.pose;
    }

    public Pose2d getBestVisionPose() {
        LimelightHelpers.PoseEstimate best = null;

        for (CameraID cam : CameraID.values()) {
            LimelightHelpers.PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam.name); // [web:63]
            if (!isEstimateUsable(est)) {
                continue;
            }
            if (best == null || isBetter(est, best)) {
                best = est;
            }
        }

        if (best == null) {
            return null;
        }
        return best.pose;
    }

    public void updatePoseEstimator() {
        LimelightHelpers.PoseEstimate best = chooseBestEstimate();
        if (best == null) {
            return;
        }

        Pose2d pose2d = best.pose;
        double timestampSec = best.timestampSeconds;

        m_poseEstimatorConsumer.addVisionMeasurement(pose2d, timestampSec);
    }

    // ------------------------
    // Internal helpers
    // ------------------------
    private LimelightHelpers.PoseEstimate chooseBestEstimate() {
        LimelightHelpers.PoseEstimate best = null;

        for (CameraID cam : CameraID.values()) {
            LimelightHelpers.PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam.name); // [web:63]
            if (!isEstimateUsable(est)) {
                continue;
            }
            if (best == null || isBetter(est, best)) {
                best = est;
            }
        }
        return best;
    }

    private boolean isEstimateUsable(LimelightHelpers.PoseEstimate est) {
        if (est == null)
            return false;
        if (est.tagCount < m_minTagCount)
            return false;
        if (est.rawFiducials == null || est.rawFiducials.length == 0)
            return false;

        // Reject if robot is spinning too quickly or driving too fast
        if (Math.abs(m_angularVelocityDegPerSec) > 360.0)
            return false;
        if (m_linearVelocityMps > 4.0)
            return false;

        // Simple per-tag ambiguity and distance checks using MT2 per-fiducial data.
        for (var f : est.rawFiducials) {
            if (f.ambiguity > m_maxAmbiguity)
                return false;
            if (f.distToCamera > m_maxDistanceMeters)
                return false;
        }

        return true;
    }

    private boolean isBetter(LimelightHelpers.PoseEstimate cand, LimelightHelpers.PoseEstimate best) {
        if (cand.tagCount != best.tagCount) {
            return cand.tagCount > best.tagCount;
        }
        if (cand.avgTagArea != best.avgTagArea) {
            return cand.avgTagArea > best.avgTagArea;
        }
        return cand.avgTagDist < best.avgTagDist;
    }
    
    public double getLastRightLatencyMs() {
        return LimelightHelpers.getLatency_Capture("limelight-right")
                + LimelightHelpers.getLatency_Pipeline("limelight-right");
    }

    public Rotation2d getFieldHeadingFromVision(CameraID camera) {
        LimelightHelpers.PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera.name);
        if (!isEstimateUsable(est)) {
            return null;
        }
        return est.pose.getRotation();
    }

    @Override
    public void periodic() {
        if (!Constants.USE_DEBUGGING)
            return;
        
        SmartDashboard.putNumber("Time_Elapsed", DriverStation.getMatchTime());
        SmartDashboard.putBoolean("Has_Any_Targets", getBestVisionPose() != null);
        boolean isRedAlliance = !DriverStation.getAlliance().isEmpty() && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
        SmartDashboard.putNumber("Target_Distance", TurretAimMath.solveForBasket(m_poseEstimatorConsumer.getEstimatedPosition(), isRedAlliance).distanceMeters());
    }
}