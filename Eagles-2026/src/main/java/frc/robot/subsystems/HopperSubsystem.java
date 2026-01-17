package frc.robot.subsystems;

import static frc.robot.Constants.Hopper.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class HopperSubsystem extends SubsystemBase {
  private final SparkMax m_motor = new SparkMax(kMotorCanId, MotorType.kBrushless);
  private final DigitalInput m_ballSensor = new DigitalInput(kBallSensorDio);

  private double clamp(double v) {
    return MathUtil.clamp(v, -1.0, 1.0);
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

  // ---------------- Auto/indexing  ----------------
  public Command cmdIndexToSensor() {
    return run(() -> {
      if (hasBall()) {
        stop();
      } else {
        setPercent(kIndexPercent);
      }
    }).withName("Hopper.IndexToSensor");
  }
}