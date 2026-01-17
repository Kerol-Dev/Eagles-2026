package frc.robot.subsystems;

import static frc.robot.Constants.Shooter.*;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TurretAimMath;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class ShooterSubsystem extends SubsystemBase {

    // ---------------- Turret (Kraken X60) ----------------
    private final TalonFX m_turret = new TalonFX(kTurretKrakenCanId);
    private final PositionDutyCycle m_turretReq = new PositionDutyCycle(0);
    private double m_turretZeroDegOffset = 0.0;
    private double m_turretTargetDeg = 0.0;

    // ---------------- Shooter flywheel (double Kraken) ----------------
    private final TalonFX m_shooterTop = new TalonFX(kShooterTopKrakenCanId);
    private final TalonFX m_shooterBottom = new TalonFX(kShooterBottomKrakenCanId);
    private final VelocityVoltage m_shooterVelReq = new VelocityVoltage(0);
    private double m_shooterTargetRpm = 0.0;

    // ---------------- Hood (TalonSRX brushed + quadrature encoder)
    // ----------------
    private final WPI_TalonSRX m_hoodMotor = new WPI_TalonSRX(kHoodTalonSrxCanId);
    private final Encoder m_hoodEnc = new Encoder(kHoodEncDioA, kHoodEncDioB);
    private double m_hoodTargetDeg = kHoodMinDeg;

    private final InterpolatingDoubleTreeMap m_hoodAngleByDistance = new InterpolatingDoubleTreeMap();

    public ShooterSubsystem() {
        configureTurret();
        configureShooter();
        configureHood();

        initHoodInterpolation();
        zeroTurretAtEnable();
        zeroHoodEncoderAtEnable();
    }

    // ---------------- Configuration ----------------
    private void configureTurret() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = kTurretP;
        cfg.Slot0.kI = kTurretI;
        cfg.Slot0.kD = kTurretD;
        m_turret.getConfigurator().apply(cfg);
    }

    private void configureShooter() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = kShooterP;
        cfg.Slot0.kI = kShooterI;
        cfg.Slot0.kD = kShooterD;
        cfg.Slot0.kV = kShooterV;
        m_shooterTop.getConfigurator().apply(cfg);
        m_shooterBottom.getConfigurator().apply(cfg);

        m_shooterBottom.setControl(new Follower(m_shooterTop.getDeviceID(), false));
    }

    private void configureHood() {
        m_hoodMotor.setNeutralMode(NeutralMode.Brake);

        double degPerPulse = (360.0 / kHoodEncoderPulsesPerRev) / kHoodEncoderToHoodGearRatio;
        m_hoodEnc.setDistancePerPulse(degPerPulse);
    }

    private void initHoodInterpolation() {
        // Example data points (distance in meters -> hood angle in degrees)
        m_hoodAngleByDistance.put(1.5, 18.0);
        m_hoodAngleByDistance.put(2.0, 22.0);
        m_hoodAngleByDistance.put(3.0, 30.0);
        m_hoodAngleByDistance.put(4.0, 39.0);
        m_hoodAngleByDistance.put(5.0, 48.0);
    }

    public void zeroTurretAtEnable() {
        m_turretZeroDegOffset = getTurretAngleDegRaw();
        m_turretTargetDeg = 0.0;
    }

    public void zeroHoodEncoderAtEnable() {
        m_hoodEnc.reset();
    }

    // ---------------- Conversions ----------------
    private static double turretDegToMotorRot(double turretDeg) {
        return (turretDeg / 360.0) * kTurretGearRatio;
    }

    private static double motorRotToTurretDeg(double motorRot) {
        return (motorRot / kTurretGearRatio) * 360.0;
    }

    private static double rpmToRps(double rpm) {
        return rpm / 60.0;
    }

    // ---------------- Turret state + safety ----------------

    private double getTurretAngleDegRaw() {
        return motorRotToTurretDeg(m_turret.getPosition().getValueAsDouble());
    }

    public double getTurretAngleDeg() {
        return getTurretAngleDegRaw() - m_turretZeroDegOffset;
    }

    public double getTurretTargetDeg() {
        return m_turretTargetDeg;
    }

    public boolean atTurretTarget() {
        return Math.abs(getTurretAngleDeg() - m_turretTargetDeg) <= kTurretToleranceDeg;
    }

    private static double clampTurretDeg(double deg) {
        return MathUtil.clamp(deg, kTurretMinDeg, kTurretMaxDeg);
    }

    public void setTurretAngleDeg(double desiredDeg) {
        double safe = clampTurretDeg(desiredDeg);
        m_turretTargetDeg = safe;

        double rawTargetDeg = safe + m_turretZeroDegOffset;
        double motorRotTarget = turretDegToMotorRot(rawTargetDeg);

        m_turret.setControl(m_turretReq.withPosition(motorRotTarget));
    }

    public void nudgeTurretDeg(double deltaDeg) {
        setTurretAngleDeg(getTurretTargetDeg() + deltaDeg);
    }

    // ---------------- Shooter RPM ----------------
    public void setShooterRpm(double rpm) {
        m_shooterTargetRpm = rpm;
        m_shooterTop.setControl(m_shooterVelReq.withVelocity(rpmToRps(rpm)));
    }

    public void stopShooter() {
        setShooterRpm(0.0);
    }

    public double getShooterTargetRpm() {
        return m_shooterTargetRpm;
    }

    public double getShooterTopRpm() {
        return m_shooterTop.getVelocity().getValueAsDouble() * 60.0;
    }

    public boolean atShooterSpeed() {
        return Math.abs(getShooterTopRpm() - m_shooterTargetRpm) <= kShooterToleranceRpm;
    }

    // ---------------- Hood angle ----------------
    public double getHoodAngleDeg() {
        return m_hoodEnc.getDistance();
    }

    public double getHoodTargetDeg() {
        return m_hoodTargetDeg;
    }

    public boolean atHoodTarget() {
        return Math.abs(getHoodAngleDeg() - m_hoodTargetDeg) <= kHoodToleranceDeg;
    }

    public void setHoodAngleDeg(double desiredDeg) {
        m_hoodTargetDeg = MathUtil.clamp(desiredDeg, kHoodMinDeg, kHoodMaxDeg);
    }

    public void setHoodForDistanceMeters(Supplier<Pose2d> getRobotPose, Supplier<Boolean> isRed) {
        setHoodAngleDeg(m_hoodAngleByDistance
                .get(TurretAimMath.solveForBasket(getRobotPose.get(), isRed.get()).distanceMeters()));
    }

    private void updateHoodControl() {
        double errorDeg = m_hoodTargetDeg - getHoodAngleDeg();
        double out = MathUtil.clamp(errorDeg * kHoodP, -kHoodMaxPercent, kHoodMaxPercent);
        m_hoodMotor.set(ControlMode.PercentOutput, out);
    }

    // ---------------- Commands ----------------
    public Command cmdZeroTurret() {
        return runOnce(this::zeroTurretAtEnable).withName("Shooter.ZeroTurret");
    }

    public Command aimAtTarget(Supplier<Pose2d> getRobotPose, Supplier<Boolean> isRed) {
        return run(() -> {
            var target = TurretAimMath.solveForBasket(getRobotPose.get(), isRed.get());
            setTurretAngleDeg(target.turretYawRelativeToRobot().getDegrees());
            setHoodForDistanceMeters(getRobotPose, isRed);
        }).withName("Shooter.AimAtTarget");
    }

    public Command cmdSetTurretDeg(double deg) {
        return runOnce(() -> setTurretAngleDeg(deg)).withName("Shooter.SetTurretDeg(" + deg + ")");
    }

    public Command cmdNudgeTurretDeg(double deltaDeg) {
        return runOnce(() -> nudgeTurretDeg(deltaDeg)).withName("Shooter.NudgeTurretDeg(" + deltaDeg + ")");
    }

    public Command cmdSetShooterRpm(double rpm) {
        return runOnce(() -> setShooterRpm(rpm)).withName("Shooter.SetRPM(" + rpm + ")");
    }

    public Command cmdStopShooter() {
        return runOnce(this::stopShooter).withName("Shooter.StopShooter");
    }

    public Command cmdSetHoodDeg(double deg) {
        return runOnce(() -> setHoodAngleDeg(deg)).withName("Shooter.SetHoodDeg(" + deg + ")");
    }

    @Override
    public void periodic() {
        updateHoodControl();

        if (!Constants.USE_DEBUGGING)
            return;

        SmartDashboard.putBoolean("Aimed_At_Target", atTurretTarget() && atHoodTarget());
        SmartDashboard.putNumber("Turret_Angle_Deg", getTurretAngleDeg());
        SmartDashboard.putNumber("Turret_Target_Deg", getTurretTargetDeg());
        SmartDashboard.putNumber("Shooter_Top_RPM", getShooterTopRpm());
        SmartDashboard.putNumber("Shooter_Target_RPM", getShooterTargetRpm());
        SmartDashboard.putNumber("Hood_Angle_Deg", getHoodAngleDeg());
        SmartDashboard.putNumber("Hood_Target_Deg", getHoodTargetDeg());
    }
}