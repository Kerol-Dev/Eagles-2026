package frc.robot.subsystems;

import static frc.robot.Constants.Climb.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage; // Use MotionMagic
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimbSubsystem extends SubsystemBase {
    private final TalonFX m_elevator = new TalonFX(kElevatorKrakenCanId);
    private final DutyCycleOut m_duty = new DutyCycleOut(0.0);

    private final MotionMagicVoltage m_mmControl = new MotionMagicVoltage(0.0);

    public ClimbSubsystem() {
        configureMotor();
        zeroPosition();
    }

    private void configureMotor() {
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
    }

    public void zeroPosition() {
        m_elevator.setPosition(0.0);
    }

    public double getPositionRot() {
        return m_elevator.getPosition().getValueAsDouble();
    }

    public boolean atPosition(double targetRotations) {
        return Math.abs(getPositionRot() - targetRotations) < kPositionTolerance;
    }

    public void setPercent(double percent) {
        m_elevator.setControl(m_duty.withOutput(MathUtil.clamp(percent, -1.0, 1.0)));
    }

    public void setPositionMM(double targetRotations) {
        m_elevator.setControl(m_mmControl.withPosition(targetRotations));
    }

    public void stop() {
        setPercent(0.0);
    }

    // --- Commands ---

    public Command cmdGoToPosition(double targetRotations) {
        return runOnce(() -> setPositionMM(targetRotations));
    }

    public Command cmdUp(double percent) {
        final double out = Math.abs(percent) * kMaxUpPercent;
        return runEnd(() -> setPercent(out), this::stop).withName("Climb.Up");
    }

    public Command cmdDown(double percent) {
        final double out = -Math.abs(percent) * kMaxDownPercent;
        return runEnd(() -> setPercent(out), this::stop).withName("Climb.Down");
    }

    public Command cmdStop() {
        return runOnce(this::stop).withName("Climb.Stop");
    }

    @Override
    public void periodic() {
        if (!Constants.USE_DEBUGGING)
            return;
        SmartDashboard.putNumber("Climb/Pos_Rot", getPositionRot());
        SmartDashboard.putNumber("Climb/Velocity_RPS", m_elevator.getVelocity().getValueAsDouble());
    }
}