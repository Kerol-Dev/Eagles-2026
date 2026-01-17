package frc.robot.subsystems;

import static frc.robot.Constants.Climb.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class ClimbSubsystem extends SubsystemBase {
    private final TalonFX m_elevator = new TalonFX(kElevatorKrakenCanId);
    private final DutyCycleOut m_duty = new DutyCycleOut(0.0);

    public ClimbSubsystem() {
        applySoftLimits();
        zeroPosition();
    }

    private void applySoftLimits() {
        var limits = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withReverseSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(kMaxRot)
                .withReverseSoftLimitThreshold(kMinRot);

        var cfg = new TalonFXConfiguration().withSoftwareLimitSwitch(limits);
        m_elevator.getConfigurator().apply(cfg);
    }

    public void zeroPosition() {
        m_elevator.setPosition(0.0);
    }

    public double getPositionRot() {
        return m_elevator.getPosition().getValueAsDouble();
    }

    private double clampPercent(double percent) {
        return MathUtil.clamp(percent, -1.0, 1.0);
    }

    public void setPercent(double percent) {
        m_elevator.setControl(m_duty.withOutput(clampPercent(percent)));
    }

    public void stop() {
        setPercent(0.0);
    }

    public Command cmdUp(double percent) {
        final double out = clampPercent(Math.abs(percent)) * kMaxUpPercent;
        return runEnd(() -> setPercent(out), this::stop).withName("Climb.Up(" + out + ")");
    }

    public Command cmdDown(double percent) {
        final double out = -clampPercent(Math.abs(percent)) * kMaxDownPercent;
        return runEnd(() -> setPercent(out), this::stop).withName("Climb.Down(" + (-out) + ")");
    }

    public Command cmdStop() {
        return runOnce(this::stop).withName("Climb.Stop");
    }

    @Override
    public void periodic() {
        if (!Constants.USE_DEBUGGING)
            return;
        SmartDashboard.putNumber("Elevator_Pos", getPositionRot());
    }
}