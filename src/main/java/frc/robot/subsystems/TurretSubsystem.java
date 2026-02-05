package frc.robot.subsystems;

import static frc.robot.Constants.Shooter.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap; // Linear curve tool
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.TargetUtils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TurretSubsystem extends SubsystemBase {

    // ---------------- Turret (Kraken X60) ----------------
    private final TalonFX m_turret = new TalonFX(kTurretKrakenCanId);
    private final MotionMagicVoltage m_turretReq = new MotionMagicVoltage(0);
    private double m_turretZeroDegOffset = 0.0;
    private double m_turretTargetDeg = 0.0;
    private boolean m_shooterEnabled = false;

    // ---------------- Hood (TalonSRX brushed) ----------------
    private final WPI_TalonSRX m_hoodMotor = new WPI_TalonSRX(kHoodTalonSrxCanId);
    private final Encoder m_hoodEnc = new Encoder(kHoodEncDioA, kHoodEncDioB);
    private double m_hoodTargetDeg = kHoodMinDeg;

    // ---------------- Interpolation Tables (The Curves) ----------------
    private final InterpolatingDoubleTreeMap m_hoodAngleByDistance = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap m_shooterRpmByDistance = new InterpolatingDoubleTreeMap();

    public TurretSubsystem() {
        configureTurret();
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
        cfg.CurrentLimits.SupplyCurrentLimit = kTurretCurrentLimitA;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_turret.getConfigurator().apply(cfg);
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

    public void setTurretAngleDeg(double desiredDeg, double feedforwardVel) {
        m_turretTargetDeg = MathUtil.clamp(desiredDeg, kTurretMinDeg, kTurretMaxDeg);
        double motorRotTarget = turretDegToMotorRot(m_turretTargetDeg + m_turretZeroDegOffset);
        m_turret.setControl(m_turretReq.withPosition(motorRotTarget).withFeedForward(feedforwardVel));
    }

    public boolean lockedAtTarget() {
        return Math.abs(getTurretAngleDeg() - m_turretTargetDeg) <= kTurretToleranceDeg
                && Math.abs(getHoodAngleDeg() - m_hoodTargetDeg) <= kHoodToleranceDeg;
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
    public void aimAtTarget(Pose2d robotPose, boolean isRed, ChassisSpeeds robotVSpeeds) {
        Translation2d target = isRed ? FieldConstants.kHubPoseRed.getTranslation() : FieldConstants.kHubPoseBlue.getTranslation();
        var targetInfo = TargetUtils.solve(robotPose, target, robotVSpeeds);
        double distance = targetInfo.effectiveDistance();

        setTurretAngleDeg(targetInfo.turretYaw().getDegrees(), targetInfo.turretFF());
        double autoHoodDeg = m_hoodAngleByDistance.get(distance);
        setHoodAngleDeg(m_shooterEnabled ? autoHoodDeg : kHoodMinDeg);

        
        Logger.recordOutput("Turret/Auto_Target_Distance", distance);
    }

    // ---------------- Commands ----------------
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
            Logger.recordOutput("Turret/Hood_Deg", getHoodAngleDeg());
            Logger.recordOutput("Turret/Hood_Deg_Target", m_hoodTargetDeg);
            Logger.recordOutput("Turret/Turret_Deg", getTurretAngleDeg());
            Logger.recordOutput("Turret/Turret_Deg_Target", m_turretTargetDeg);
        }
    }
}