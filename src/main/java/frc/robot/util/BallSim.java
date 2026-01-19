package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Timer;

public class BallSim {
    private double startTime = 0;
    private boolean isShooting = false;

    private final double CYCLE_TIME = 0.75; 
    private final double BALL_SPACING = 0.2;
    private final double ARC_HEIGHT = 2;

    public void setShooting(boolean shoot) {
        if (shoot && !isShooting) {
            startTime = Timer.getFPGATimestamp();
        }
        this.isShooting = shoot;
    }

    private final StructArrayPublisher<Pose3d> ballPublisher = 
    NetworkTableInstance.getDefault()
        .getStructArrayTopic("SimulatedBalls", Pose3d.struct)
        .publish();

    public void update(Pose3d robotPose, Pose3d targetPose) {
        if (!isShooting) {
            ballPublisher.set(new Pose3d[0]);
            return;
        }

        double elapsed = Timer.getFPGATimestamp() - startTime;
        Pose3d[] ballPoses = new Pose3d[3];

        for (int i = 0; i < 3; i++) {
            double t = ((elapsed - (i * BALL_SPACING)) % CYCLE_TIME) / CYCLE_TIME;
            
            if (t < 0) {
                ballPoses[i] = robotPose;
            } else {
                ballPoses[i] = calculateBezierPose(robotPose, targetPose, t);
            }
        }
        ballPublisher.set(ballPoses);
    }

    private Pose3d calculateBezierPose(Pose3d start, Pose3d end, double t) {
        Translation3d p0 = start.getTranslation();
        Translation3d p2 = end.getTranslation();
        
        Translation3d p1 = new Translation3d(
                (p0.getX() + p2.getX()) * 0.5,
                (p0.getY() + p2.getY()) * 0.5,
                (p0.getZ() + p2.getZ()) * 0.5 + ARC_HEIGHT
        );

        double oneMinusT = 1 - t;
        double w0 = oneMinusT * oneMinusT;
        double w1 = 2 * oneMinusT * t;
        double w2 = t * t;

        double x = p0.getX() * w0 + p1.getX() * w1 + p2.getX() * w2;
        double y = p0.getY() * w0 + p1.getY() * w1 + p2.getY() * w2;
        double z = p0.getZ() * w0 + p1.getZ() * w1 + p2.getZ() * w2;

        Translation3d currentPos = new Translation3d(x, y, z);

        return new Pose3d(currentPos, new Rotation3d());
    }
}