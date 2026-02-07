package frc.robot.subsystems;

import static frc.robot.Constants.Shooter.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.util.TargetUtils;

public class ShooterSubsystem extends SubsystemBase {
    // ---------------- Shooter flywheel (double Kraken) ----------------
    private final TalonFX m_shooterRight = new TalonFX(kShooterTopKrakenCanId);
    private final TalonFX m_shooterLeft = new TalonFX(kShooterBottomKrakenCanId);

    private final VelocityVoltage m_shooterVelReq = new VelocityVoltage(0.0).withSlot(0);

    private double m_shooterTargetRpm = 0.0;
    private boolean m_shooterEnabled = false;

    // ---------------- Interpolation Tables (The Curves) ----------------
    // distance (m) -> shooter RPM
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

        m_shooterRight.getConfigurator().apply(cfg);
        m_shooterLeft.getConfigurator().apply(cfg);

        m_shooterLeft.setControl(
            new Follower(m_shooterRight.getDeviceID(), MotorAlignmentValue.Opposed)
        );
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

        if (!m_shooterEnabled) {
            return;
        }

        double targetVelRps = targetRpm / 60.0;
        m_shooterRight.setControl(m_shooterVelReq.withVelocity(targetVelRps));
    }

    public void setShooterEnabled(boolean enabled) {
        m_shooterEnabled = enabled;

        if (!enabled) {
            // Zero target and actively stop when disabled.
            m_shooterTargetRpm = 0.0;
            m_shooterRight.stopMotor();
        } else {
            // Re-apply the last target RPM when re-enabled.
            setShooterRpm(m_shooterTargetRpm);
        }
    }

    public boolean atShooterSpeed() {
        return Math.abs(getShooterTopRpm() - m_shooterTargetRpm) <= kShooterToleranceRpm;
    }

    public double getShooterTopRpm() {
        return m_shooterRight.getVelocity().getValueAsDouble() * 60.0;
    }

    // ---------------- Aiming ----------------
    public void setAutoRPM(Pose2d robotPose, boolean isRed, ChassisSpeeds robotVSpeeds) {
        Translation2d target = TargetUtils.solveTargetPose(robotPose, isRed).getTranslation();
        var targetInfo = TargetUtils.solve(robotPose, target, robotVSpeeds);
        double distance = targetInfo.effectiveDistance();

        double autoRpm = m_shooterRpmByDistance.get(distance);

        setShooterRpm(autoRpm);

        Logger.recordOutput("Shooter/AutoRPM", autoRpm);
    }

    // ---------------- Commands ----------------
    public Command cmdStopShooter() {
        return runOnce(
                () -> {
                    setShooterEnabled(false);
                })
                .withName("Shooter.Stop");
    }

    public Command cmdEnableShooter(boolean enable) {
        return runOnce(() -> setShooterEnabled(enable))
                .withName("Shooter.Enable(" + enable + ")");
    }

    /**
     * Example: manual spin-up to a given RPM and wait until at speed.
     */
    public Command cmdSpinUpAndWait(double rpm) {
        return cmdEnableShooter(true)
                .andThen(runOnce(() -> setShooterRpm(rpm)))
                .andThen(new WaitUntilCommand(this::atShooterSpeed))
                .withName("Shooter.SpinUpAndWait(" + rpm + ")");
    }


    public Command cmdManualShooterCommand() {
        return cmdEnableShooter(true)
                .andThen(new WaitUntilCommand(this::atShooterSpeed))
                .withName("Shooter.ManualWait");
    }

    @Override
    public void periodic() {
        if (Constants.USE_DEBUGGING) {
            Logger.recordOutput("Shooter/Actual_RPM", getShooterTopRpm());
            Logger.recordOutput("Shooter/Target_RPM", m_shooterTargetRpm);
        }
    }
}