package frc.robot.subsystems;

import static frc.robot.Constants.Climb.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimbSubsystem extends SubsystemBase {

    // --- Motors ---
    // 1. Lead Screw ("Sonsuz Civata") - Moves elevator mechanism forward/backward
    private final SparkMax m_leadScrew = new SparkMax(kLeadScrewCanId, SparkMax.MotorType.kBrushless);
    private final SparkClosedLoopController m_leadScrewCtrl = m_leadScrew.getClosedLoopController();

    // 2. Elevator Motors - Separate Left/Right control
    private final SparkMax m_elevatorLeft = new SparkMax(kElevatorLeftCanId, SparkMax.MotorType.kBrushless);
    private final SparkClosedLoopController m_elevatorLeftCtrl = m_elevatorLeft.getClosedLoopController();

    private final SparkMax m_elevatorRight = new SparkMax(kElevatorRightCanId, SparkMax.MotorType.kBrushless);
    private final SparkClosedLoopController m_elevatorRightCtrl = m_elevatorRight.getClosedLoopController();

    public ClimbSubsystem() {
        configureMotors();
        zeroPositions();
    }

    private void configureMotors() {
        // --- Lead Screw Configuration (Standard PID) ---
        SparkMaxConfig leadScrewCfg = new SparkMaxConfig();
        leadScrewCfg.closedLoop.pid(kLeadScrewP, kLeadScrewI, kLeadScrewD, ClosedLoopSlot.kSlot0);
        leadScrewCfg.closedLoop.outputRange(-1, 1);
        
        leadScrewCfg.softLimit.forwardSoftLimitEnabled(true);
        leadScrewCfg.softLimit.forwardSoftLimit(kLeadScrewForwardLimit);
        leadScrewCfg.softLimit.reverseSoftLimitEnabled(true);
        leadScrewCfg.softLimit.reverseSoftLimit(kLeadScrewReverseLimit);
        leadScrewCfg.smartCurrentLimit(kLeadScrewCurrentLimit);

        m_leadScrew.configure(leadScrewCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // --- Elevator Configuration (MAXMotion / Smooth Position) ---
        SparkMaxConfig elevatorCfg = new SparkMaxConfig();
        
        // PID + Motion Profiling Parameters
        elevatorCfg.closedLoop.pid(kElevatorP, kElevatorI, kElevatorD, ClosedLoopSlot.kSlot0);
        elevatorCfg.closedLoop.maxMotion.cruiseVelocity(kElevatorMaxVel);
        elevatorCfg.closedLoop.maxMotion.maxAcceleration(kElevatorMaxAccel);
        elevatorCfg.closedLoop.maxMotion.allowedProfileError(kElevatorTolerance);
        elevatorCfg.closedLoop.outputRange(-1, 1);

        // Limits
        elevatorCfg.softLimit.forwardSoftLimitEnabled(true);
        elevatorCfg.softLimit.forwardSoftLimit(kElevatorForwardLimit);
        elevatorCfg.softLimit.reverseSoftLimitEnabled(true);
        elevatorCfg.softLimit.reverseSoftLimit(kElevatorReverseLimit);
        elevatorCfg.smartCurrentLimit(kElevatorCurrentLimit);

        // Apply config to both elevators
        m_elevatorLeft.configure(elevatorCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_elevatorRight.configure(elevatorCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void zeroPositions() {
        m_leadScrew.getEncoder().setPosition(0.0);
        m_elevatorLeft.getEncoder().setPosition(0.0);
        m_elevatorRight.getEncoder().setPosition(0.0);
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

    public double getLeftElevatorPos() {
        return m_elevatorLeft.getEncoder().getPosition();
    }

    public double getRightElevatorPos() {
        return m_elevatorRight.getEncoder().getPosition();
    }

    public boolean atLeftElevatorPos(double target) {
        return Math.abs(getLeftElevatorPos() - target) < kElevatorTolerance;
    }

    public boolean atRightElevatorPos(double target) {
        return Math.abs(getRightElevatorPos() - target) < kElevatorTolerance;
    }

    public void setLeftElevatorPosition(double targetRot) {
        m_elevatorLeftCtrl.setSetpoint(targetRot, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }

    public void setRightElevatorPosition(double targetRot) {
        m_elevatorRightCtrl.setSetpoint(targetRot, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    }

    public void setElevatorPercent(double percent) {
        double clamped = MathUtil.clamp(percent, -1.0, 1.0);
        m_elevatorLeft.set(clamped);
        m_elevatorRight.set(clamped);
    }

    public void stop() {
        m_leadScrew.stopMotor();
        m_elevatorLeft.stopMotor();
        m_elevatorRight.stopMotor();
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

    // Elevator Commands (Separate Control)
    public Command cmdLeftElevatorToPos(double targetRot) {
        return run(() -> setLeftElevatorPosition(targetRot))
                .until(() -> atLeftElevatorPos(targetRot))
                .withName("Climb.LeftElevToPos");
    }

    public Command cmdRightElevatorToPos(double targetRot) {
        return run(() -> setRightElevatorPosition(targetRot))
                .until(() -> atRightElevatorPos(targetRot))
                .withName("Climb.RightElevToPos");
    }

    // Moves both elevators to the same setpoint simultaneously
    public Command cmdBothElevatorsToPos(double targetRot) {
        return run(() -> {
            setLeftElevatorPosition(targetRot);
            setRightElevatorPosition(targetRot);
        })
        .until(() -> atLeftElevatorPos(targetRot) && atRightElevatorPos(targetRot))
        .withName("Climb.BothElevToPos");
    }

    public Command cmdElevatorsManual(double percent) {
        return runEnd(() -> setElevatorPercent(percent), () -> setElevatorPercent(0.0))
                .withName("Climb.ElevManual");
    }

    public Command cmdStop() {
        return runOnce(this::stop).withName("Climb.Stop");
    }

    @Override
    public void periodic() {
        if (!Constants.USE_DEBUGGING) return;
        SmartDashboard.putNumber("Climb/LeadScrew/Pos", getLeadScrewPos());
        SmartDashboard.putNumber("Climb/ElevLeft/Pos", getLeftElevatorPos());
        SmartDashboard.putNumber("Climb/ElevRight/Pos", getRightElevatorPos());
    }
}