package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

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

    private static final double kSingleTagStdDev = 1.0; 
    private static final double kMultiTagStdDev = 0.5;

    private int m_minTagCount = 1;
    private double m_maxAmbiguity = 0.5;
    private double m_maxDistanceMeters = 5.0;

    private double m_angularVelocityDegPerSec = 0.0;
    private double m_linearVelocityMps = 0.0;

    public VisionSubsystem(SwerveDrivePoseEstimator poseEstimatorConsumer) {
        m_poseEstimatorConsumer = poseEstimatorConsumer;
        m_poseEstimatorConsumer.setVisionMeasurementStdDevs(
                VecBuilder.fill(kSingleTagStdDev, kSingleTagStdDev, 9999999.0));
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

    public void updatePoseEstimator() {
        if (RobotContainer.passingBump) return; // Skip if unstable

        List<LimelightHelpers.PoseEstimate> validEstimates = new ArrayList<>();

        for (CameraID cam : CameraID.values()) {
            LimelightHelpers.PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cam.name);
            if (isEstimateUsable(est)) {
                validEstimates.add(est);
            }
        }

        if (validEstimates.isEmpty()) return;
        
        double avgX = 0;
        double avgY = 0;
        double avgTime = 0;
        double minAvgDist = 999.0;
        int totalTags = 0;

        for (LimelightHelpers.PoseEstimate est : validEstimates) {
            avgX += est.pose.getX();
            avgY += est.pose.getY();
            avgTime += est.timestampSeconds;
            
            // Track metrics for standard deviation calculation
            if (est.avgTagDist < minAvgDist) minAvgDist = est.avgTagDist;
            totalTags += est.tagCount;
        }

        avgX /= validEstimates.size();
        avgY /= validEstimates.size();
        avgTime /= validEstimates.size();

        Rotation2d gyroRotation = validEstimates.get(0).pose.getRotation(); 
        Pose2d averagedPose = new Pose2d(avgX, avgY, gyroRotation);

        double xyStdDev = calculateStdDev(minAvgDist, totalTags);

        m_poseEstimatorConsumer.setVisionMeasurementStdDevs(
            VecBuilder.fill(xyStdDev, xyStdDev, 9999999.0) // Infinite rotation std dev (trust gyro)
        );
        
        m_poseEstimatorConsumer.addVisionMeasurement(averagedPose, avgTime);
    }

    private double calculateStdDev(double avgDist, int totalTagCount) {
        // Base trust level
        double confidence = (totalTagCount > 1) ? kMultiTagStdDev : kSingleTagStdDev;
        
        //Add 0.1m of uncertainty for every meter of distance
        confidence += (avgDist * 0.15); 
        
        // Reward very close tags: If < 1.5m, boost trust significantly
        if (avgDist < 1.5) {
            confidence *= 0.5; 
        }

        return confidence;
    }

    private boolean isEstimateUsable(LimelightHelpers.PoseEstimate est) {
        if (est == null) return false;
        if (est.tagCount < m_minTagCount) return false;
        if (est.rawFiducials == null || est.rawFiducials.length == 0) return false;

        // Reject if robot is spinning/moving too fast (motion blur)
        if (Math.abs(m_angularVelocityDegPerSec) > 360.0) return false;
        if (m_linearVelocityMps > 4.0) return false;

        // Per-tag sanity checks
        for (var f : est.rawFiducials) {
            if (f.ambiguity > m_maxAmbiguity) return false;
            if (f.distToCamera > m_maxDistanceMeters) return false;
        }
        return true;
    }

    @Override
    public void periodic() {
        if (!Constants.USE_DEBUGGING) return;
        
        boolean hasTargets = false;
        for (CameraID cam : CameraID.values()) {
             if (LimelightHelpers.getTV(cam.name)) hasTargets = true;
        }
        Logger.recordOutput("Vision/Has_Any_Targets", hasTargets);
    }
}
