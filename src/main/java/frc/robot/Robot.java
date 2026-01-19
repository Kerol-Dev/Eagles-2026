package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private static Robot instance;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public Robot() {
    instance = this;
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
    Pose3d robotPose  = new Pose3d(m_robotContainer.drivebase.getPose().getX(), m_robotContainer.drivebase.getPose().getY(), RobotContainer.climed ? 1.1 : 0.6,
        new Rotation3d(m_robotContainer.drivebase.getPose().getRotation()));
    m_robotContainer.ballSim.update(robotPose, new Pose3d(4.632, 4.039, 2, new Rotation3d()));
  }

  @Override
  public void disabledInit() {
    m_robotContainer.setMotorBrake(true);
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
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