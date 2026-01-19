package frc.robot.subsystems;

import static frc.robot.Constants.Hopper.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class HopperSubsystem extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(kMotorCanId, MotorType.kBrushless);
    private final DigitalInput m_ballSensor = new DigitalInput(kBallSensorDio);

    private final Timer m_emptyTimer = new Timer();
    private boolean m_lastHasBall = false;

    private double clamp(double v) {
        return MathUtil.clamp(v, -1.0, 1.0);
    }

    public boolean isEmptyFor2s() {
        return !hasBall() && m_emptyTimer.hasElapsed(2.0);
    }

    public void setPercent(double percent) {
        m_motor.set(clamp(percent));
    }

    public void stop() {
        setPercent(0.0);
    }

    public boolean hasBall() {
        boolean raw = m_ballSensor.get();
        return kSensorInverted ? !raw : raw;
    }

    // ---------------- Manual commands ----------------
    public Command cmdPush(double percent) {
        final double out = Math.abs(percent);
        return runEnd(() -> setPercent(out), this::stop).withName("Hopper.Push(" + out + ")");
    }

    public Command cmdReverse(double percent) {
        final double out = -Math.abs(percent);
        return runEnd(() -> setPercent(out), this::stop).withName("Hopper.Reverse(" + (-out) + ")");
    }

    public Command cmdStop() {
        return runOnce(this::stop).withName("Hopper.Stop");
    }

    // ---------------- Auto/indexing ----------------
    public Command cmdIndexToSensor() {
        return run(() -> {
            if (hasBall()) {
                stop();
            } else {
                setPercent(kIndexPercent);
            }
        }).withName("Hopper.IndexToSensor");
    }

    @Override
    public void periodic() {
        boolean nowHasBall = hasBall();

        if (nowHasBall) {
            m_emptyTimer.stop();
            m_emptyTimer.reset();
        } else {
            if (m_lastHasBall) {
                m_emptyTimer.restart();
            } else if (!m_emptyTimer.isRunning()) {
                m_emptyTimer.start();
            }
        }

        m_lastHasBall = nowHasBall;

        if (!Constants.USE_DEBUGGING)
            return;
        SmartDashboard.putBoolean("Hopper/Ball Sensor", hasBall());
        SmartDashboard.putBoolean("Hopper/Hopper Empty 2s", isEmptyFor2s());
    }
}