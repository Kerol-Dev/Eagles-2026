package frc.robot.subsystems;

import static frc.robot.Constants.VisionSim.*;

import java.nio.file.Path;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

public class PhotonVisionSimSubsystem extends edu.wpi.first.wpilibj2.command.SubsystemBase {
    private final Supplier<Pose2d> m_robotPoseSupplier;
    private final edu.wpi.first.math.estimator.SwerveDrivePoseEstimator m_poseEstimator;

    private final AprilTagFieldLayout m_tagLayout;

    private final VisionSystemSim m_visionSim;

    private final PhotonCamera m_camRight = new PhotonCamera(kCamRightName);
    private final PhotonCamera m_camLeft = new PhotonCamera(kCamLeftName);
    private final PhotonCamera m_camBack = new PhotonCamera(kCamBackName);

    private final PhotonPoseEstimator m_estRight;
    private final PhotonPoseEstimator m_estLeft;
    private final PhotonPoseEstimator m_estBack;

    public PhotonVisionSimSubsystem(
            Supplier<Pose2d> robotPoseSupplier,
            edu.wpi.first.math.estimator.SwerveDrivePoseEstimator poseEstimator) {

        m_robotPoseSupplier = robotPoseSupplier;
        m_poseEstimator = poseEstimator;

        try {
            Path deploy = Filesystem.getDeployDirectory().toPath();
            Path jsonPath = Path.of(deploy.toString(), "apriltags", "2026-rebuilt-andymark.json");

            m_tagLayout = new AprilTagFieldLayout(jsonPath);
        } catch (java.io.IOException e) {
            throw new RuntimeException(e);
        }

        m_visionSim = new VisionSystemSim(kSimSystemName);
        m_visionSim.addAprilTags(m_tagLayout);

        setupSimCamera(m_camRight, kRobotToCamRight);
        setupSimCamera(m_camLeft, kRobotToCamLeft);
        setupSimCamera(m_camBack, kRobotToCamBack);

        m_estRight = makePoseEstimator(kRobotToCamRight);
        m_estLeft = makePoseEstimator(kRobotToCamLeft);
        m_estBack = makePoseEstimator(kRobotToCamBack);

        Field2d debugField = m_visionSim.getDebugField();
        SmartDashboard.putData("PhotonSimField", debugField);
    }

    private void setupSimCamera(PhotonCamera cam, edu.wpi.first.math.geometry.Transform3d robotToCam) {
        if (!RobotBase.isSimulation())
            return;

        PhotonCameraSim camSim = new PhotonCameraSim(cam);
        m_visionSim.addCamera(camSim, robotToCam);
    }

    private PhotonPoseEstimator makePoseEstimator(edu.wpi.first.math.geometry.Transform3d robotToCam) {
        var est = new PhotonPoseEstimator(
                m_tagLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCam);

        est.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        return est;
    }

    @Override
    public void simulationPeriodic() {
        Pose2d robotPose = m_robotPoseSupplier.get();
        m_visionSim.update(robotPose);
    }

    @Override
    public void periodic() {
        Pose2d robotPose2d = m_poseEstimator.getEstimatedPosition();
        Pose3d robotPose3d = new Pose3d(robotPose2d);

        double offset = SmartDashboard.getNumber("ClimbOffset", 0.0);
        final double increment = 0.01;
        if (RobotContainer.climed) {
            offset = Math.min(0.5, offset + increment);
        } else {
            offset = Math.max(0.0, offset - increment);
        }
        SmartDashboard.putNumber("ClimbOffset", offset);

        robotPose3d = new Pose3d(
            robotPose3d.getX(),
            robotPose3d.getY(),
            robotPose3d.getZ() + offset,
            robotPose3d.getRotation()
        );

        SmartDashboard.putNumberArray("3D/RobotPose", pose3dToArray(robotPose3d));
        SmartDashboard.putNumberArray("3D/CamRight", pose3dToArray(robotPose3d.transformBy(kRobotToCamRight)));
        SmartDashboard.putNumberArray("3D/CamLeft", pose3dToArray(robotPose3d.transformBy(kRobotToCamLeft)));
        SmartDashboard.putNumberArray("3D/CamBack", pose3dToArray(robotPose3d.transformBy(kRobotToCamBack)));

        applyEstimator(m_estRight, m_camRight, robotPose2d);
        applyEstimator(m_estLeft, m_camLeft, robotPose2d);
        applyEstimator(m_estBack, m_camBack, robotPose2d);
    }

    private double[] pose3dToArray(Pose3d pose) {
        var quart = pose.getRotation().getQuaternion();
        return new double[] {
                pose.getX(),
                pose.getY(),
                pose.getZ(),
                quart.getW(),
                quart.getX(),
                quart.getY(),
                quart.getZ()
        };
    }

    private void applyEstimator(PhotonPoseEstimator estimator, PhotonCamera cam, Pose2d refPose) {
        estimator.setReferencePose(refPose);
        var pipelineResult = cam.getLatestResult();

        if (pipelineResult == null || !pipelineResult.hasTargets())
            return;

        Optional<EstimatedRobotPose> result = estimator.update(pipelineResult);
        if (result.isPresent()) {
            EstimatedRobotPose est = result.get();
            Pose2d estPose2d = est.estimatedPose.toPose2d();

            double distance = refPose.getTranslation().getDistance(estPose2d.getTranslation());
            if (distance > 1.0)
                return;

            m_poseEstimator.addVisionMeasurement(
                    estPose2d,
                    est.timestampSeconds,
                    VecBuilder.fill(kStdDevX, kStdDevY, kStdDevTheta));
        }
    }
}