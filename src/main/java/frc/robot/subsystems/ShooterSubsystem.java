package frc.robot.subsystems;

import static frc.robot.Constants.Shooter.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap; // Linear curve tool
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TurretAimMath;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ShooterSubsystem extends SubsystemBase {

    // ---------------- Turret (Kraken X60) ----------------
    private final TalonFX m_turret = new TalonFX(kTurretKrakenCanId);
    private final MotionMagicVoltage m_turretReq = new MotionMagicVoltage(0);
    private double m_turretZeroDegOffset = 0.0;
    private double m_turretTargetDeg = 0.0;

    // ---------------- Shooter flywheel (double Kraken) ----------------
    private final TalonFX m_shooterTop = new TalonFX(kShooterTopKrakenCanId);
    private final TalonFX m_shooterBottom = new TalonFX(kShooterBottomKrakenCanId);
    private final VelocityVoltage m_shooterVelReq = new VelocityVoltage(0);
    private double m_shooterTargetRpm = 0.0;
    private boolean m_shooterEnabled = false;

    // ---------------- Hood (TalonSRX brushed) ----------------
    private final WPI_TalonSRX m_hoodMotor = new WPI_TalonSRX(kHoodTalonSrxCanId);
    private final Encoder m_hoodEnc = new Encoder(kHoodEncDioA, kHoodEncDioB);
    private double m_hoodTargetDeg = kHoodMinDeg;

    // ---------------- Interpolation Tables (The Curves) ----------------
    private final InterpolatingDoubleTreeMap m_hoodAngleByDistance = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_shooterRpmByDistance = new InterpolatingDoubleTreeMap();

    public ShooterSubsystem() {
        configureTurret();
        configureShooter();
        configureHood();

        initInterpolationTables();
        zeroTurretAtEnable();
        zeroHoodEncoderAtEnable();
    }

    // ---------------- Configuration ----------------
    private void configureTurret() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = kTurretP;
        cfg.Slot0.kI = kTurretI;
        cfg.Slot0.kD = kTurretD;
        cfg.Slot0.kS = kTurretS;
        cfg.Slot0.kV = kTurretV;

        cfg.MotionMagic.MotionMagicCruiseVelocity = kTurretCruiseVelocity;
        cfg.MotionMagic.MotionMagicAcceleration = kTurretAcceleration;
        m_turret.getConfigurator().apply(cfg);
    }

    private void configureShooter() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = kShooterP;
        cfg.Slot0.kI = kShooterI;
        cfg.Slot0.kD = kShooterD;
        cfg.Slot0.kV = kShooterV;

        cfg.CurrentLimits.StatorCurrentLimit = 60;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        m_shooterTop.getConfigurator().apply(cfg);
        m_shooterBottom.getConfigurator().apply(cfg);

        m_shooterBottom.setControl(new Follower(m_shooterTop.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    private void configureHood() {
        m_hoodMotor.setNeutralMode(NeutralMode.Brake);
        double degPerPulse = (360.0 / kHoodEncoderPulsesPerRev) / kHoodEncoderToHoodGearRatio;
        m_hoodEnc.setDistancePerPulse(degPerPulse);
    }

    private void initInterpolationTables() {
        // --- Hood Curve (Meters -> Degrees) ---
        m_hoodAngleByDistance.put(1.5, 18.0);
        m_hoodAngleByDistance.put(5.0, 48.0);

        // --- RPM Curve (Meters -> RPM) ---
        m_shooterRpmByDistance.put(1.5, 3000.0); // Close shot
        m_shooterRpmByDistance.put(3.0, 3500.0); // Mid shot
        m_shooterRpmByDistance.put(5.0, 4200.0); // Long shot
    }

    // ---------------- Turret Logic ----------------
    private static double turretDegToMotorRot(double turretDeg) {
        return (turretDeg / 360.0) * kTurretGearRatio;
    }

    private static double motorRotToTurretDeg(double motorRot) {
        return (motorRot / kTurretGearRatio) * 360.0;
    }

    public double getTurretAngleDeg() {
        return motorRotToTurretDeg(m_turret.getPosition().getValueAsDouble()) - m_turretZeroDegOffset;
    }

    public void setTurretAngleDeg(double desiredDeg) {
        m_turretTargetDeg = MathUtil.clamp(desiredDeg, kTurretMinDeg, kTurretMaxDeg);
        double motorRotTarget = turretDegToMotorRot(m_turretTargetDeg + m_turretZeroDegOffset);
        m_turret.setControl(m_turretReq.withPosition(motorRotTarget));
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

    public boolean lockedAtTarget() {
        return Math.abs(getTurretAngleDeg() - m_turretTargetDeg) <= kTurretToleranceDeg
                && Math.abs(getHoodAngleDeg() - m_hoodTargetDeg) <= kHoodToleranceDeg;
    }

    public double getShooterTopRpm() {
        return m_shooterTop.getVelocity().getValueAsDouble() * 60.0;
    }

    // ---------------- Hood Logic ----------------
    public double getHoodAngleDeg() {
        return m_hoodEnc.getDistance();
    }

    public void setHoodAngleDeg(double desiredDeg) {
        m_hoodTargetDeg = MathUtil.clamp(desiredDeg, kHoodMinDeg, kHoodMaxDeg);
    }

    private void updateHoodControl() {
        double errorDeg = m_hoodTargetDeg - getHoodAngleDeg();
        double out = MathUtil.clamp(errorDeg * kHoodP, -kHoodMaxPercent, kHoodMaxPercent);
        m_hoodMotor.set(ControlMode.PercentOutput, out);
    }

    // ---------------- Aiming ----------------
    public void aimAtTarget(Pose2d robotPose, boolean isRed) {
        var targetInfo = TurretAimMath.solveForBasket(robotPose, isRed);
        double distance = targetInfo.distanceMeters();

        // 1. Aim Turret
        setTurretAngleDeg(targetInfo.turretYawRelativeToRobot().getDegrees());

        // 2. Set RPM from Linear Curve
        double autoRpm = m_shooterRpmByDistance.get(distance);
        setShooterRpm(autoRpm);

        // 3. Set Hood Angle from Linear Curve
        double autoHoodDeg = m_hoodAngleByDistance.get(distance);
        setHoodAngleDeg(m_shooterEnabled ? autoHoodDeg : kHoodMinDeg);

        SmartDashboard.putNumber("Shooter/Auto_Target_Distance", distance);
    }

    // ---------------- Commands ----------------
    public Command cmdStopShooter() {
        return runOnce(() -> setShooterRpm(0)).withName("Shooter.Stop");
    }

    public Command cmdEnableShooter(boolean enable) {
        return runOnce(() -> setShooterEnabled(enable)).withName("Shooter.Enable(" + enable + ")");
    }

    public void zeroTurretAtEnable() {
        m_turretZeroDegOffset = motorRotToTurretDeg(m_turret.getPosition().getValueAsDouble());
    }

    public void zeroHoodEncoderAtEnable() {
        m_hoodEnc.reset();
    }

    @Override
    public void periodic() {
        updateHoodControl();

        if (Constants.USE_DEBUGGING) {
            SmartDashboard.putNumber("Shooter/Actual_RPM", getShooterTopRpm());
            SmartDashboard.putNumber("Shooter/Target_RPM", m_shooterTargetRpm);
            SmartDashboard.putNumber("Shooter/Hood_Deg", getHoodAngleDeg());
            SmartDashboard.putNumber("Shooter/Hood_Deg_Target", m_hoodTargetDeg);
            SmartDashboard.putNumber("Shooter/Turret_Deg", getTurretAngleDeg());
            SmartDashboard.putNumber("Shooter/Turret_Deg_Target", m_turretTargetDeg);
        }
    }
}