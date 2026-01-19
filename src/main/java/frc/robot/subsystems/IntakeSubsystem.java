package frc.robot.subsystems;

import static frc.robot.Constants.Intake.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX m_angleMotor = new TalonFX(kAngleMotorCanId);
    private final SparkMax m_rollerMotor = new SparkMax(kRollerMotorCanId, MotorType.kBrushless);

    // Closed loop controllers
    private final MotionMagicVoltage m_angleControl = new MotionMagicVoltage(0);
    private final SparkClosedLoopController m_rollerCL = m_rollerMotor.getClosedLoopController();

    // Encoders
    private final RelativeEncoder m_rollerEncoder = m_rollerMotor.getEncoder();

    // Cached targets
    private double m_angleTargetDeg = kClosedAngleDeg;
    private double m_rollerTargetRpm = 0.0;

    public IntakeSubsystem() {
        configureAngleMotor();
        configureRollerMotor();
    }

    private void configureAngleMotor() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.Slot0.kP = kAngleP;
        cfg.Slot0.kI = kAngleI;
        cfg.Slot0.kD = kAngleD;
        cfg.Slot0.kS = kAngleS;
        cfg.Slot0.kV = kAngleV;
        cfg.MotionMagic.MotionMagicCruiseVelocity = kAngleCruiseVelocity;
        cfg.MotionMagic.MotionMagicAcceleration = kAngleAcceleration;

        cfg.CurrentLimits.SupplyCurrentLimit = kAngleCurrentLimitA;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        cfg.Feedback.SensorToMechanismRatio = 1.0;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_angleMotor.getConfigurator().apply(cfg);
    }

    @SuppressWarnings("removal")
    private void configureRollerMotor() {
        SparkMaxConfig cfg = new SparkMaxConfig();

        cfg.closedLoop
                .p(kRollerP)
                .i(kRollerI)
                .d(kRollerD)
                .velocityFF(kRollerFF)
                .outputRange(kRollerMinOut, kRollerMaxOut);

        cfg.smartCurrentLimit(kRollerCurrentLimitA);

        m_rollerMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getAngleDeg() {
        double motorRotations = m_angleMotor.getPosition().getValueAsDouble();
        return (motorRotations / kGearboxRatio) * 360.0;
    }

    public double getAngleTargetDeg() {
        return m_angleTargetDeg;
    }

    public boolean atAngleTarget() {
        return Math.abs(getAngleDeg() - m_angleTargetDeg) <= kAngleToleranceDeg;
    }

    public double getRollerRpm() {
        return m_rollerEncoder.getVelocity();
    }

    public void setAngleTargetDeg(double targetDeg) {
        double targetRotations = (targetDeg / 360.0) * kGearboxRatio;

        m_angleMotor.setControl(m_angleControl.withPosition(targetRotations));
    }

    public void setRollerTargetRpm(double rpm) {
        m_rollerTargetRpm = rpm;
        m_rollerCL.setSetpoint(rpm, ControlType.kVelocity);
    }

    public void stopRoller() {
        m_rollerTargetRpm = 0.0;
        m_rollerMotor.stopMotor();
    }

    public void stopAngle() {
        m_angleMotor.stopMotor();
    }

    // --- Commands ---

    public Command cmdOpen() {
        return runOnce(() -> setAngleTargetDeg(kOpenAngleDeg))
                .andThen(cmdRunRollerRpm(3000))
                .withName("Intake.Open");
    }

    public Command cmdClose() {
        return runOnce(() -> setAngleTargetDeg(kClosedAngleDeg))
                .andThen(cmdStopRoller())
                .withName("Intake.Close");
    }

    public Command cmdSetAngleDeg(double targetDeg) {
        return runOnce(() -> setAngleTargetDeg(targetDeg))
                .withName("Intake.SetAngleDeg(" + targetDeg + ")");
    }

    public Command cmdRunRollerRpm(double rpm) {
        return runEnd(
                () -> setRollerTargetRpm(rpm),
                this::stopRoller)
                .withName("Intake.RunRollerRpm(" + rpm + ")");
    }

    public Command cmdStopRoller() {
        return runOnce(this::stopRoller).withName("Intake.StopRoller");
    }

    @Override
    public void periodic() {
        if (!Constants.USE_DEBUGGING)
            return;

        SmartDashboard.putBoolean("Intake/Intake_Is_Open",
                getAngleDeg() >= (kOpenAngleDeg + kClosedAngleDeg) / 2);
        SmartDashboard.putNumber("Intake/Intake_Angle_Deg", getAngleDeg());
        SmartDashboard.putNumber("Intake/Intake_Angle_Target_Deg", getAngleTargetDeg());
        SmartDashboard.putNumber("Intake/Intake_Roller_RPM", getRollerRpm());
        SmartDashboard.putNumber("Intake/Intake_Roller_RPM_Target", m_rollerTargetRpm);
    }
}