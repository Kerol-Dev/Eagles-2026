package frc.robot.subsystems;

import static frc.robot.Constants.Climb.*;

import org.littletonrobotics.junction.Logger;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    // ----------------- Hardware -----------------
    private final SparkMax leadScrew = new SparkMax(kLeadScrewCanId, SparkMax.MotorType.kBrushless);
    private final SparkClosedLoopController leadScrewCtrl = leadScrew.getClosedLoopController();

    private final TalonFX elevator = new TalonFX(kElevatorRightCanId);

    // ----------------- Construction -----------------
    public ClimbSubsystem() {
        configureLeadScrew();
        configureElevator();
        zeroPositions();
    }

    private void configureLeadScrew() {
        SparkMaxConfig cfg = new SparkMaxConfig();

        cfg.closedLoop.pid(kLeadScrewP, kLeadScrewI, kLeadScrewD, ClosedLoopSlot.kSlot0);
        cfg.closedLoop.outputRange(-1.0, 1.0);

        cfg.softLimit.forwardSoftLimitEnabled(true);
        cfg.softLimit.forwardSoftLimit(kLeadScrewForwardLimit);
        cfg.softLimit.reverseSoftLimitEnabled(true);
        cfg.softLimit.reverseSoftLimit(kLeadScrewReverseLimit);

        cfg.smartCurrentLimit(kLeadScrewCurrentLimit);

        leadScrew.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureElevator() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.Slot0.kP = kElevatorP;
        cfg.Slot0.kI = kElevatorI;
        cfg.Slot0.kD = kElevatorD;

        cfg.MotionMagic.MotionMagicCruiseVelocity = kElevatorMaxVel;
        cfg.MotionMagic.MotionMagicAcceleration = kElevatorMaxAccel;

        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kElevatorForwardLimit;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kElevatorReverseLimit;

        cfg.CurrentLimits.SupplyCurrentLimit = kElevatorCurrentLimit;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        elevator.getConfigurator().apply(cfg);
    }

    // ----------------- Zeroing -----------------
    public void zeroPositions() {
        leadScrew.getEncoder().setPosition(0.0);
        elevator.setPosition(0.0);
    }

    // ----------------- Lead screw API -----------------
    public double getLeadScrewPosition() {
        return leadScrew.getEncoder().getPosition();
    }

    public boolean isLeadScrewHome() {
        return Math.abs(getLeadScrewPosition()) < kLeadScrewTolerance;
    }

    public boolean isLeadScrewEngaged() {
        return Math.abs(getLeadScrewPosition() - kElevatorLeadEngagedSetpoint) < kLeadScrewTolerance;
    }

    public void setLeadScrewPosition(double targetRot) {
        leadScrewCtrl.setSetpoint(targetRot, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void setLeadScrewPercent(double percent) {
        leadScrew.set(MathUtil.clamp(percent, -1.0, 1.0));
    }

    // ----------------- Elevator API -----------------
    public double getElevatorPosition() {
        return elevator.getPosition().getValueAsDouble();
    }

    public boolean isElevatorUp() {
        return Math.abs(getElevatorPosition() - kElevatorUpSetpoint) < kElevatorTolerance;
    }

    public boolean isElevatorDown() {
        return Math.abs(getElevatorPosition()) < kElevatorTolerance;
    }

    public void setElevatorUp() {
        elevator.setControl(new MotionMagicVoltage(kElevatorUpSetpoint));
    }

    public void setElevatorDown() {
        elevator.setControl(new MotionMagicVoltage(kElevatorDownSetpoint));
    }

    public void setElevatorPercent(double percent) {
        double clamped = MathUtil.clamp(percent, -1.0, 1.0);
        elevator.setControl(new DutyCycleOut(clamped));
    }

    // ----------------- Shared controls -----------------
    public void stop() {
        leadScrew.stopMotor();
        elevator.stopMotor();
    }

    // ----------------- Command factories -----------------
    // Lead screw commands
    public Command leadScrewEngage() {
        return run(() -> setLeadScrewPosition(kElevatorLeadEngagedSetpoint))
                .until(() -> isLeadScrewEngaged())
                .withName("Climb.LeadScrewEngage");
    }

    public Command leadScrewHome() {
        return run(() -> setLeadScrewPosition(0))
                .until(() -> isLeadScrewHome())
                .withName("Climb.LeadScrewHome");
    }

    public Command leadScrewManual(double percent) {
        return runEnd(() -> setLeadScrewPercent(percent),
                () -> setLeadScrewPercent(0.0))
                .withName("Climb.LeadScrewManual");
    }

    // Elevator commands
    public Command elevatorUp() {
        return run(() -> setElevatorUp())
                .until(() -> isElevatorUp())
                .withName("Climb.ElevUp");
    }

    public Command elevatorDown() {
        return run(() -> setElevatorDown())
                .until(() -> isElevatorDown())
                .withName("Climb.ElevDown");
    }

    public Command elevatorManual(double percent) {
        return runEnd(() -> setElevatorPercent(percent),
                () -> setElevatorPercent(0.0))
                .withName("Climb.ElevManual");
    }

    public Command stopCommand() {
        return runOnce(this::stop).withName("Climb.Stop");
    }

    // ----------------- Telemetry -----------------
    @Override
    public void periodic() {
        if (!Constants.USE_DEBUGGING) {
            return;
        }

        Logger.recordOutput("Climb/LeadScrew/Pos", getLeadScrewPosition());
        Logger.recordOutput("Climb/Elev/Pos", getElevatorPosition());
    }
}