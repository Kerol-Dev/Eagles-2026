package frc.robot.subsystems;

import static frc.robot.Constants.Intake.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkClosedLoopController;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.RelativeEncoder;

public class IntakeSubsystem extends SubsystemBase {
    // Hardware
    private final SparkFlex m_angleMotor = new SparkFlex(kAngleMotorCanId, MotorType.kBrushless);
    private final SparkMax m_rollerMotor = new SparkMax(kRollerMotorCanId, MotorType.kBrushless);

    // Closed loop controllers
    private final SparkClosedLoopController m_angleCL = m_angleMotor.getClosedLoopController();
    private final SparkClosedLoopController m_rollerCL = m_rollerMotor.getClosedLoopController();

    // Encoders
    private final RelativeEncoder m_angleEncoder = m_angleMotor.getEncoder();
    private final RelativeEncoder m_rollerEncoder = m_rollerMotor.getEncoder();

    // Cached targets
    private double m_angleTargetDeg = kClosedAngleDeg;
    private double m_rollerTargetRpm = 0.0;

    public IntakeSubsystem() {
        configureAngleMotor();
        configureRollerMotor();
    }

    private void configureAngleMotor() {
        SparkFlexConfig cfg = new SparkFlexConfig();

        // Closed-loop PID for position control on the Flex
        cfg.closedLoop
                .p(kAngleP)
                .i(kAngleI)
                .d(kAngleD)
                .outputRange(kAngleMinOut, kAngleMaxOut);

        cfg.smartCurrentLimit(kAngleCurrentLimitA);

        cfg.encoder.positionConversionFactor(kAngleDegPerMotorRotation);

        // Apply configuration and persist it on the controller
        m_angleMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureRollerMotor() {
        SparkMaxConfig cfg = new SparkMaxConfig();

        // Velocity closed-loop for RPM control [web:31][web:43]
        cfg.closedLoop
                .p(kRollerP)
                .i(kRollerI)
                .d(kRollerD)
                .velocityFF(kRollerFF)
                .outputRange(kRollerMinOut, kRollerMaxOut);

        cfg.smartCurrentLimit(kRollerCurrentLimitA);

        m_rollerMotor.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getAngleDeg() {
        return m_angleEncoder.getPosition();
    }

    public double getAngleTargetDeg() {
        return m_angleTargetDeg;
    }

    public boolean atAngleTarget() {
        return Math.abs(getAngleDeg() - m_angleTargetDeg) <= kAngleToleranceDeg;
    }

    public double getRollerRpm() {
        return m_rollerEncoder.getVelocity();
    }

    public double getRollerTargetRpm() {
        return m_rollerTargetRpm;
    }

    public void setAngleTargetDeg(double targetDeg) {
        m_angleTargetDeg = targetDeg;
        m_angleCL.setReference(targetDeg, ControlType.kPosition);
    }

    public void setRollerTargetRpm(double rpm) {
        m_rollerTargetRpm = rpm;
        m_rollerCL.setReference(rpm, ControlType.kVelocity);
    }

    public void stopRoller() {
        m_rollerTargetRpm = 0.0;
        m_rollerMotor.stopMotor();
    }

    public Command cmdOpen() {
        return runOnce(() -> setAngleTargetDeg(kOpenAngleDeg)).andThen(cmdRunRollerRpm(3000))
                .withName("Intake.Open");
    }

    public Command cmdClose() {
        return runOnce(() -> setAngleTargetDeg(kClosedAngleDeg)).andThen(cmdStopRoller())
                .withName("Intake.Close");
    }

    public Command cmdSetAngleDeg(double targetDeg) {
        return runOnce(() -> setAngleTargetDeg(targetDeg))
                .withName("Intake.SetAngleDeg(" + targetDeg + ")");
    }

    public Command cmdRunRollerRpm(double rpm) {
        return runEnd(
                () -> setRollerTargetRpm(rpm),
                this::stopRoller)
                .withName("Intake.RunRollerRpm(" + rpm + ")");
    }

    public Command cmdStopRoller() {
        return runOnce(this::stopRoller).withName("Intake.StopRoller");
    }
}