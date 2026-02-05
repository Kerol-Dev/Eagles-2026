package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {

  private static Robot instance;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public Robot() {
    instance = this;

    Logger.recordMetadata("Eagles2026", "2026");

    Logger.addDataReceiver(new WPILOGWriter());
    Logger.addDataReceiver(new NT4Publisher());

    Logger.start();
  }

  public static Robot getInstance() {
    return instance;
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Update the ball sim with the robot's current pose and the target pose
    Pose3d robotPose = new Pose3d(m_robotContainer.drivebase.getPose().getX(),
        m_robotContainer.drivebase.getPose().getY(), RobotContainer.climbed ? 1.1 : 0.6,
        new Rotation3d(m_robotContainer.drivebase.getPose().getRotation()));
    Pose3d targetPose = !DriverStation.getAlliance().isEmpty() && DriverStation.getAlliance().get() == Alliance.Blue
        ? new Pose3d(4.632, 4.039, 2, new Rotation3d())
        : new Pose3d(11.932, 4.039, 2, new Rotation3d());
    m_robotContainer.ballSim.update(robotPose, targetPose);
  }

  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
    RobotContainer.climbed = false;
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}