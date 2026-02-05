package frc.robot.subsystems;

import static frc.robot.Constants.Shooter.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.SimpleTurretAim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class ShooterSubsystem extends SubsystemBase {
    // ---------------- Shooter flywheel (double Kraken) ----------------
    private final TalonFX m_shooterTop = new TalonFX(kShooterTopKrakenCanId);
    private final TalonFX m_shooterBottom = new TalonFX(kShooterBottomKrakenCanId);
    private final VelocityVoltage m_shooterVelReq = new VelocityVoltage(0);
    private double m_shooterTargetRpm = 0.0;
    private boolean m_shooterEnabled = false;

    // ---------------- Interpolation Tables (The Curves) ----------------
    private final InterpolatingDoubleTreeMap m_shooterRpmByDistance = new InterpolatingDoubleTreeMap();

    public ShooterSubsystem() {
        configureShooter();

        initInterpolationTables();
    }

    // ---------------- Configuration ----------------
    private void configureShooter() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = kShooterP;
        cfg.Slot0.kI = kShooterI;
        cfg.Slot0.kD = kShooterD;
        cfg.Slot0.kV = kShooterV;

        cfg.CurrentLimits.StatorCurrentLimit = kShooterCurrentLimitA;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        m_shooterTop.getConfigurator().apply(cfg);
        m_shooterBottom.getConfigurator().apply(cfg);

        m_shooterBottom.setControl(new Follower(m_shooterTop.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    private void initInterpolationTables() {
        // --- RPM Curve (Meters -> RPM) ---
        m_shooterRpmByDistance.put(1.5, 3000.0); // Close shot
        m_shooterRpmByDistance.put(3.0, 3500.0); // Mid shot
        m_shooterRpmByDistance.put(5.0, 4200.0); // Long shot
    }

    // ---------------- Shooter Logic ----------------
    public void setShooterRpm(double targetRpm) {
        m_shooterTargetRpm = targetRpm;
        if (m_shooterEnabled) {
            double targetVel = targetRpm / 60.0; // RPM to RPS
            m_shooterTop.setControl(m_shooterVelReq.withVelocity(targetVel));
        }
    }

    public void setShooterEnabled(boolean enabled) {
        m_shooterEnabled = enabled;
        if (!enabled) {
            setShooterRpm(0.0);
        }
    }

    public boolean atShooterSpeed() {
        return Math.abs(getShooterTopRpm() - m_shooterTargetRpm) <= kShooterToleranceRpm;
    }

    public double getShooterTopRpm() {
        return m_shooterTop.getVelocity().getValueAsDouble() * 60.0;
    }

    // ---------------- Aiming ----------------
    public void setAutoRPM(Pose2d robotPose, boolean isRed, ChassisSpeeds robotVSpeeds) {
        Translation2d target = isRed ? FieldConstants.kHubPoseRed.getTranslation()
                : FieldConstants.kHubPoseBlue.getTranslation();
        var targetInfo = SimpleTurretAim.solve(robotPose, target, robotVSpeeds, 0.0, 0.0);
        double distance = targetInfo.distance();
        double autoRpm = m_shooterRpmByDistance.get(distance) + targetInfo.shooterRpmAdjust();
        setShooterRpm(autoRpm);
    }

    // ---------------- Commands ----------------
    public Command cmdStopShooter() {
        return runOnce(() -> setShooterRpm(0)).withName("Shooter.Stop");
    }

    public Command cmdEnableShooter(boolean enable) {
        return runOnce(() -> setShooterEnabled(enable)).withName("Shooter.Enable(" + enable + ")");
    }

    public Command cmdManualShooterCommand() {
        return Commands.runOnce(() -> cmdEnableShooter(true), this)
                .andThen(new WaitUntilCommand(() -> atShooterSpeed()));
    }

    @Override
    public void periodic() {

        if (Constants.USE_DEBUGGING) {
            Logger.recordOutput("Shooter/Actual_RPM", getShooterTopRpm());
            Logger.recordOutput("Shooter/Target_RPM", m_shooterTargetRpm);
        }
    }
}