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

    // --- Motors ---
    private final SparkMax m_leadScrew = new SparkMax(kLeadScrewCanId, SparkMax.MotorType.kBrushless);
    private final SparkClosedLoopController m_leadScrewCtrl = m_leadScrew.getClosedLoopController();

    private final TalonFX m_elevator = new TalonFX(kElevatorRightCanId);

    public ClimbSubsystem() {
        configureMotors();
        zeroPositions();
    }

    private void configureMotors() {
        SparkMaxConfig leadScrewCfg = new SparkMaxConfig();
        leadScrewCfg.closedLoop.pid(kLeadScrewP, kLeadScrewI, kLeadScrewD, ClosedLoopSlot.kSlot0);
        leadScrewCfg.closedLoop.outputRange(-1, 1);

        leadScrewCfg.softLimit.forwardSoftLimitEnabled(true);
        leadScrewCfg.softLimit.forwardSoftLimit(kLeadScrewForwardLimit);
        leadScrewCfg.softLimit.reverseSoftLimitEnabled(true);
        leadScrewCfg.softLimit.reverseSoftLimit(kLeadScrewReverseLimit);
        leadScrewCfg.smartCurrentLimit(kLeadScrewCurrentLimit);

        m_leadScrew.configure(leadScrewCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        TalonFXConfiguration elevatorCfg = new TalonFXConfiguration();
        elevatorCfg.Slot0.kP = kElevatorP;
        elevatorCfg.Slot0.kI = kElevatorI;
        elevatorCfg.Slot0.kD = kElevatorD;
        elevatorCfg.MotionMagic.MotionMagicCruiseVelocity = kElevatorMaxVel;
        elevatorCfg.MotionMagic.MotionMagicAcceleration = kElevatorMaxAccel;
        elevatorCfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        elevatorCfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kElevatorForwardLimit;
        elevatorCfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        elevatorCfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kElevatorReverseLimit;
        elevatorCfg.CurrentLimits.SupplyCurrentLimit = kElevatorCurrentLimit;
        elevatorCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_elevator.getConfigurator().apply(elevatorCfg);
    }

    public void zeroPositions() {
        m_leadScrew.getEncoder().setPosition(0.0);
        m_elevator.setPosition(0.0);
    }

    // --- Lead Screw Methods ---

    public double getLeadScrewPos() {
        return m_leadScrew.getEncoder().getPosition();
    }

    public boolean atLeadScrewPos(double target) {
        return Math.abs(getLeadScrewPos() - target) < kLeadScrewTolerance;
    }

    public void setLeadScrewPosition(double targetRot) {
        m_leadScrewCtrl.setSetpoint(targetRot, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void setLeadScrewPercent(double percent) {
        m_leadScrew.set(MathUtil.clamp(percent, -1.0, 1.0));
    }

    // --- Elevator Methods ---

    public double getElevatorPos() {
        return m_elevator.getPosition().getValueAsDouble();
    }

    public boolean atElevatorPos(double target) {
        return Math.abs(getElevatorPos() - target) < kElevatorTolerance;
    }

    public void setElevatorPosition(double targetRot) {
        m_elevator.setControl(new MotionMagicVoltage(targetRot));
    }

    public void setElevatorPercent(double percent) {
        double clamped = MathUtil.clamp(percent, -1.0, 1.0);
        m_elevator.setControl(new DutyCycleOut(clamped));
    }

    public void stop() {
        m_leadScrew.stopMotor();
        m_elevator.stopMotor();
    }

    // --- Commands ---

    // Lead Screw Commands
    public Command cmdLeadScrewToPos(double targetRot) {
        return run(() -> setLeadScrewPosition(targetRot))
                .until(() -> atLeadScrewPos(targetRot))
                .withName("Climb.LeadScrewToPos");
    }

    public Command cmdLeadScrewManual(double percent) {
        return runEnd(() -> setLeadScrewPercent(percent), () -> setLeadScrewPercent(0.0))
                .withName("Climb.LeadScrewManual");
    }

    // Elevator Commands

    public Command cmdElevatorToPos(double targetRot) {
        return run(() -> setElevatorPosition(targetRot))
                .until(() -> atElevatorPos(targetRot))
                .withName("Climb.ElevToPos");
    }

    public Command cmdElevatorManual(double percent) {
        return runEnd(() -> setElevatorPercent(percent), () -> setElevatorPercent(0.0))
                .withName("Climb.ElevManual");
    }

    public Command cmdStop() {
        return runOnce(this::stop).withName("Climb.Stop");
    }

    @Override
    public void periodic() {
        if (!Constants.USE_DEBUGGING)
            return;
        SmartDashboard.putNumber("Climb/LeadScrew/Pos", getLeadScrewPos());
        SmartDashboard.putNumber("Climb/Elev/Pos", getElevatorPos());
    }
}