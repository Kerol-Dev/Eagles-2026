package frc.robot.subsystems;

import static frc.robot.Constants.Hopper.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {

  // ----------------- Hardware -----------------
  private final SparkMax indexerMotor = new SparkMax(kIndexerMotorCanId, MotorType.kBrushless);
  private final DigitalInput ballSensor = new DigitalInput(kBallSensorDio);

  // ----------------- State -----------------
  private final Timer emptyTimer = new Timer();
  private boolean lastHasBall = false;

  // ----------------- Helpers -----------------
  private double clampPercent(double value) {
    return MathUtil.clamp(value, -1.0, 1.0);
  }

  // ----------------- Public API -----------------
  public boolean hasBall() {
    boolean raw = ballSensor.get();
    return kSensorInverted ? !raw : raw;
  }

  public boolean isEmptyFor2s() {
    return !hasBall() && emptyTimer.hasElapsed(2.0);
  }

  public void setPercent(double percent) {
    indexerMotor.set(clampPercent(percent));
  }

  public void stop() {
    setPercent(0.0);
  }

  // ----------------- Command factories (manual) -----------------
  public Command push(double percent) {
    final double out = Math.abs(percent);
    return runEnd(() -> setPercent(out), this::stop)
        .withName("Hopper.Push(" + out + ")");
  }

  public Command reverse(double percent) {
    final double out = -Math.abs(percent);
    return runEnd(() -> setPercent(out), this::stop)
        .withName("Hopper.Reverse(" + (-out) + ")");
  }

  public Command stopCommand() {
    return runOnce(this::stop).withName("Hopper.Stop");
  }

  // ----------------- Command factories (auto/indexing) -----------------
  public Command indexToSensor() {
    return run(() -> {
      if (hasBall()) {
        stop();
      } else {
        setPercent(kIndexPercent);
      }
    }).withName("Hopper.IndexToSensor");
  }

  // ----------------- Periodic -----------------
  @Override
  public void periodic() {
    boolean nowHasBall = hasBall();

    if (nowHasBall) {
      emptyTimer.stop();
      emptyTimer.reset();
    } else {
      if (lastHasBall) {
        emptyTimer.restart();
      } else if (!emptyTimer.isRunning()) {
        emptyTimer.start();
      }
    }

    lastHasBall = nowHasBall;

    if (!Constants.USE_DEBUGGING) {
      return;
    }

    Logger.recordOutput("Hopper/BallSensor", hasBall());
    Logger.recordOutput("Hopper/EmptyFor2s", isEmptyFor2s());
  }
}