package frc.robot.subsystems;

import static frc.robot.Constants.Intake.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  // ----------------- Hardware -----------------
  private final TalonFX angleMotor = new TalonFX(kAngleMotorCanId);
  private final SparkMax rollerMotor = new SparkMax(kRollerMotorCanId, MotorType.kBrushless);

  // Closed-loop controllers
  private final MotionMagicVoltage angleRequest = new MotionMagicVoltage(0.0);
  private final SparkClosedLoopController rollerController = rollerMotor.getClosedLoopController();

  // Encoders
  private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();

  // Cached targets
  private double angleTargetDeg = kClosedAngleDeg;
  private double rollerTargetRpm = 0.0;

  // ----------------- Construction -----------------
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

    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    angleMotor.getConfigurator().apply(cfg);
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

    rollerMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ----------------- Angle API -----------------
  public double getAngleDeg() {
    double motorRotations = angleMotor.getPosition().getValueAsDouble();
    return (motorRotations / kGearboxRatio) * 360.0;
  }

  public double getAngleTargetDeg() {
    return angleTargetDeg;
  }

  public boolean atAngleTarget() {
    return Math.abs(getAngleDeg() - angleTargetDeg) <= kAngleToleranceDeg;
  }

  public void setAngleTargetDeg(double targetDeg) {
    angleTargetDeg = targetDeg;
    double targetRotations = (targetDeg / 360.0) * kGearboxRatio;
    angleMotor.setControl(angleRequest.withPosition(targetRotations));
  }

  public void stopAngle() {
    angleMotor.stopMotor();
  }

  // ----------------- Roller API -----------------
  public double getRollerRpm() {
    return rollerEncoder.getVelocity();
  }

  public void setRollerTargetRpm(double rpm) {
    rollerTargetRpm = rpm;
    rollerController.setSetpoint(rpm, ControlType.kVelocity);
  }

  public void stopRoller() {
    rollerTargetRpm = 0.0;
    rollerMotor.stopMotor();
  }

  // ----------------- Command factories -----------------
  public Command open() {
    return runOnce(() -> setAngleTargetDeg(kOpenAngleDeg))
        .andThen(new WaitUntilCommand(this::atAngleTarget))
        .andThen(runRollerRpm(kIntakeRollerRPM))
        .withName("Intake.Open");
  }

  public Command close() {
    return runOnce(() -> setAngleTargetDeg(kClosedAngleDeg))
        .andThen(stopRollerCommand())
        .andThen(new WaitUntilCommand(this::atAngleTarget))
        .withName("Intake.Close");
  }

  public Command setAngleDegCommand(double targetDeg) {
    return runOnce(() -> setAngleTargetDeg(targetDeg))
        .withName("Intake.SetAngleDeg(" + targetDeg + ")");
  }

  public Command runRollerRpm(double rpm) {
    return runEnd(
            () -> setRollerTargetRpm(rpm),
            this::stopRoller)
        .withName("Intake.RunRollerRpm(" + rpm + ")");
  }

  public Command stopRollerCommand() {
    return runOnce(this::stopRoller).withName("Intake.StopRoller");
  }

  // ----------------- Telemetry -----------------
  @Override
  public void periodic() {
    if (!Constants.USE_DEBUGGING) {
      return;
    }

    boolean isOpen = getAngleDeg() >= (kOpenAngleDeg + kClosedAngleDeg) / 2.0;

    Logger.recordOutput("Intake/IsOpen", isOpen);
    Logger.recordOutput("Intake/AngleDeg", getAngleDeg());
    Logger.recordOutput("Intake/AngleTargetDeg", getAngleTargetDeg());
    Logger.recordOutput("Intake/RollerRPM", getRollerRpm());
    Logger.recordOutput("Intake/RollerRPMTarget", rollerTargetRpm);
  }
}