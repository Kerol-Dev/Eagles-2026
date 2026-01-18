package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final double MAX_SPEED = Units.feetToMeters(15); // 15 ft/s
    public static final boolean USE_DEBUGGING = true;

    public static class OperatorConstants {
        public static final double DEADBAND = 0.1;
    }

    public static final class Intake {
        // CAN IDs
        public static final int kAngleMotorCanId = 21; // SparkFlex + NEO Vortex
        public static final int kRollerMotorCanId = 22; // SparkMax + NEO 1.1

        // Angle setpoints
        public static final double kOpenAngleDeg = 70.0;
        public static final double kClosedAngleDeg = 5.0;

        // Angle closed-loop gains
        public static final double kAngleP = 0.08;
        public static final double kAngleI = 0.0;
        public static final double kAngleD = 0.0;
        public static final double kAngleMinOut = -0.5;
        public static final double kAngleMaxOut = 0.5;

        public static final double kAngleMotorRotationsPerMechanismRev = 50.0;
        public static final double kAngleDegPerMotorRotation = 360.0 / kAngleMotorRotationsPerMechanismRev;

        // Angle safety/tolerance
        public static final double kAngleToleranceDeg = 2.0;

        // Roller velocity closed-loop gains
        public static final double kRollerP = 0.0002;
        public static final double kRollerI = 0.0;
        public static final double kRollerD = 0.0;
        public static final double kRollerFF = 0.00018;
        public static final double kRollerMinOut = -1.0;
        public static final double kRollerMaxOut = 1.0;

        // Current limits
        public static final int kAngleCurrentLimitA = 60;
        public static final int kRollerCurrentLimitA = 60;
    }

    public static final class Shooter {
        // CAN IDs
        public static final int kTurretKrakenCanId = 31;

        public static final int kShooterTopKrakenCanId = 32;
        public static final int kShooterBottomKrakenCanId = 33;

        public static final int kHoodTalonSrxCanId = 41;

        // Hood quadrature encoder DIO channels (A/B)
        public static final int kHoodEncDioA = 0;
        public static final int kHoodEncDioB = 1;

        // ---------------- Turret ----------------
        public static final double kTurretGearRatio = 30.0;

        // Cable-safe window relative to zero
        public static final double kTurretMinDeg = -90.0;
        public static final double kTurretMaxDeg = 200.0;

        // Turret PID
        public static final double kTurretP = 60.0;
        public static final double kTurretI = 0.0;
        public static final double kTurretD = 2.0;

        // Turret tolerances
        public static final double kTurretToleranceDeg = 1.5;

        // ---------------- Shooter flywheel ----------------
        public static final double kShooterP = 0.15;
        public static final double kShooterI = 0.0;
        public static final double kShooterD = 0.0;
        public static final double kShooterV = 0.12; // FF for VelocityVoltage

        public static final double kShooterToleranceRpm = 75.0;

        // ---------------- Hood ----------------
        public static final double kHoodEncoderPulsesPerRev = 7.0;
        public static final double kHoodEncoderToHoodGearRatio = 10.0;

        public static final double kHoodMinDeg = 10.0;
        public static final double kHoodMaxDeg = 60.0;

        // Hood P control (percent output) + clamp
        public static final double kHoodP = 0.025;
        public static final double kHoodMaxPercent = 0.35;
        public static final double kHoodToleranceDeg = 1.0;
    }

    public static final class Climb {
        public static final int kElevatorKrakenCanId = 51;

        // Soft limits in MOTOR ROTATIONS
        public static final double kMinRot = 0.0;
        public static final double kMaxRot = 120.0;

        public static final double kMaxUpPercent = 1.0;
        public static final double kMaxDownPercent = 1.0;
    }

    public static final class Hopper {
        public static final int kMotorCanId = 61;
        public static final int kBallSensorDio = 2;

        public static final double kIndexPercent = 0.25;
        public static final double kReversePercent = -0.25;

        public static final boolean kSensorInverted = true;
    }

    public static final class VisionSim {
        public static final String kSimSystemName = "main";

        public static final String kCamRightName = "photon-right";
        public static final String kCamLeftName = "photon-left";
        public static final String kCamBackName = "photon-back";

        public static final Transform3d kRobotToCamRight = new Transform3d(new Translation3d(0.25, -0.20, 0.5),
                new Rotation3d(0, 0, Math.toRadians(-40)));
        public static final Transform3d kRobotToCamLeft = new Transform3d(new Translation3d(0.25, 0.20, 0.5),
                new Rotation3d(0, 0, Math.toRadians(40)));
        public static final Transform3d kRobotToCamBack = new Transform3d(new Translation3d(-0.25, 0.0, 0.5),
                new Rotation3d(0, 0, Math.toRadians(180)));

        // Basic trust (tune later)
        public static final double kStdDevX = 0.2;
        public static final double kStdDevY = 0.2;
        public static final double kStdDevTheta = 2.0;

    }
}