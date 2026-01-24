package frc.robot.subsystems;

import static frc.robot.Constants.Climb.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimbSubsystem extends SubsystemBase {
    private final TalonFX m_elevator = new TalonFX(kElevatorKrakenCanId);
    private final DutyCycleOut m_duty = new DutyCycleOut(0.0);
    private final MotionMagicVoltage m_mmControl = new MotionMagicVoltage(0.0);

    private final SparkMax m_elevator_push = new SparkMax(kElevatorPushKrakenCanId, SparkMax.MotorType.kBrushless);
    private final SparkClosedLoopController m_elevator_push_cl = m_elevator_push.getClosedLoopController();

    public ClimbSubsystem() {
        configureMotors();
        zeroPosition();
    }

    private void configureMotors() {
        // --- TalonFX Configuration (Main Elevator) ---
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.SoftwareLimitSwitch
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(kMaxRot)
                .withReverseSoftLimitThreshold(kMinRot);

        cfg.Slot0.kP = kElevatorP;
        cfg.Slot0.kI = kElevatorI;
        cfg.Slot0.kD = kElevatorD;
        cfg.Slot0.kS = kElevatorS;
        cfg.Slot0.kV = kElevatorV;

        cfg.MotionMagic.MotionMagicCruiseVelocity = kCruiseVelocity;
        cfg.MotionMagic.MotionMagicAcceleration = kAcceleration;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        m_elevator.getConfigurator().apply(cfg);

        // --- SparkMax Configuration (Push Elevator) ---
        SparkMaxConfig pushCfg = new SparkMaxConfig();
        pushCfg.closedLoop.pid(kElevatorPushP, kElevatorPushI, kElevatorPushD, ClosedLoopSlot.kSlot0);
        pushCfg.softLimit.forwardSoftLimitEnabled(true);
        pushCfg.softLimit.forwardSoftLimit(kElevatorPushForwardLimit);
        pushCfg.softLimit.reverseSoftLimitEnabled(true);
        pushCfg.softLimit.reverseSoftLimit(kElevatorPushReverseLimit);
        pushCfg.smartCurrentLimit(kElevatorPushCurrentLimit);
        
        m_elevator_push.configure(pushCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void zeroPosition() {
        m_elevator.setPosition(0.0);
        m_elevator_push.getEncoder().setPosition(0.0);
    }

    // --- Main Elevator Methods ---
    public double getElevatorPositionRot() {
        return m_elevator.getPosition().getValueAsDouble();
    }

    public boolean atElevatorPosition(double targetRotations) {
        return Math.abs(getElevatorPositionRot() - targetRotations) < kElevatorPositionTolerance;
    }

    public void setElevatorPercent(double percent) {
        m_elevator.setControl(m_duty.withOutput(MathUtil.clamp(percent, -1.0, 1.0)));
    }

    public void setElevatorPositionMM(double targetRotations) {
        m_elevator.setControl(m_mmControl.withPosition(targetRotations));
    }

    // --- Push Motor Methods ---
    public double getPushPositionRot() {
        return m_elevator_push.getEncoder().getPosition();
    }

    public boolean atPushPosition(double targetRotations) {
        return Math.abs(getPushPositionRot() - targetRotations) < kElevatorPushPositionTolerance;
    }

    public void setPushPercent(double percent) {
        m_elevator_push.set(MathUtil.clamp(percent, -1.0, 1.0));
    }

    public void setPushPosition(double targetRotations) {
        m_elevator_push_cl.setSetpoint(targetRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void stop() {
        setElevatorPercent(0.0);
        setPushPercent(0.0);
    }

    // --- Commands ---

    // Main Elevator Commands
    public Command cmdGoToPosition(double targetRotations) {
        return run(() -> setElevatorPositionMM(targetRotations))
                .until(() -> atElevatorPosition(targetRotations))
                .withName("Climb.GoToPosition");
    }

    public Command cmdUp(double percent) {
        final double out = Math.abs(percent) * kMaxUpPercent;
        return runEnd(() -> setElevatorPercent(out), () -> setElevatorPercent(0.0)).withName("Climb.Up");
    }

    public Command cmdDown(double percent) {
        final double out = -Math.abs(percent) * kMaxDownPercent;
        return runEnd(() -> setElevatorPercent(out), () -> setElevatorPercent(0.0)).withName("Climb.Down");
    }

    // Push Motor Commands
    public Command cmdPushGoToPosition(double targetRotations) {
        return run(() -> setPushPosition(targetRotations))
                .until(() -> atPushPosition(targetRotations))
                .withName("Climb.PushGoToPos");
    }

    public Command cmdPushSetPercent(double percent) {
        return runEnd(() -> setPushPercent(percent), () -> setPushPercent(0.0)).withName("Climb.PushManual");
    }

    public Command cmdStop() {
        return runOnce(this::stop).withName("Climb.Stop");
    }

    @Override
    public void periodic() {
        if (!Constants.USE_DEBUGGING)
            return;
        SmartDashboard.putNumber("Climb/Elevator/Pos_Rot", getElevatorPositionRot());
        SmartDashboard.putNumber("Climb/Push/Pos_Rot", getPushPositionRot());
    }
}
