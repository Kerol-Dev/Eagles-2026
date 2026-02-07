package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.BallSim;
import swervelib.SwerveInputStream;

public class RobotContainer {

        // Controllers
        private final CommandXboxController driverXbox = new CommandXboxController(0);

        // Subsystems
        public final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve"));
        private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
        private final HopperSubsystem hopperSubsystem = new HopperSubsystem();
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        private final TurretSubsystem turretSubsystem = new TurretSubsystem();
        private final LEDSubsystem ledSubsystem = new LEDSubsystem();

        // Simulation
        public final BallSim ballSim = new BallSim();
        public static boolean climbed = false;
        public static boolean passingBump = false;

        // State & Choosers
        private final SendableChooser<Command> autoChooser;
        private boolean m_intakeOpen = false;

        // Drive Input Stream
        private final SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
                        drivebase.getSwerveDrive(),
                        () -> -driverXbox.getLeftY(),
                        () -> -driverXbox.getLeftX())
                        .withControllerRotationAxis(driverXbox::getRightX)
                        .scaleRotation(-1)
                        .deadband(OperatorConstants.DEADBAND)
                        .allianceRelativeControl(true);

        public RobotContainer() {
                DriverStation.silenceJoystickConnectionWarning(true);
                configureBindings();
                configureEventTriggers();
                registerNamedCommands();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData(autoChooser);
        }

        private void configureBindings() {
                // Default Commands
                drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));
                hopperSubsystem.setDefaultCommand(hopperSubsystem.indexToSensor());

                turretSubsystem.setDefaultCommand(new RunCommand(() -> turretSubsystem.aimAtTarget(
                                drivebase.getPose(),
                                drivebase.isRedAlliance(),
                                drivebase.getFieldVelocity()), turretSubsystem));

                shooterSubsystem.setDefaultCommand(new RunCommand(() -> shooterSubsystem.setAutoRPM(
                                drivebase.getPose(),
                                drivebase.isRedAlliance(),
                                drivebase.getFieldVelocity()), shooterSubsystem));

                ledSubsystem.setDefaultCommand(new RunCommand(() -> ledSubsystem.updateState(
                                m_intakeOpen,
                                turretSubsystem.lockedAtTarget(),
                                shooterSubsystem.atShooterSpeed()), ledSubsystem));

                // Driver Controls
                driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));

                // Shoot: Spin up -> Wait for speed -> Feed
                driverXbox.rightTrigger()
                                .whileTrue(shooterSubsystem.cmdManualShooterCommand()
                                                .andThen(hopperSubsystem.push(0.5)))
                                .onFalse(shooterSubsystem.cmdEnableShooter(false).andThen(hopperSubsystem.stopCommand()));

                // Intake Toggle
                driverXbox.rightBumper().onTrue(
                                Commands.runOnce(() -> m_intakeOpen = !m_intakeOpen)
                                                .andThen(Commands.either(
                                                                intakeSubsystem.open(),
                                                                intakeSubsystem.close(),
                                                                () -> m_intakeOpen)));

                // Intake Reverse
                driverXbox.leftTrigger()
                                .whileTrue(intakeSubsystem.runRollerRpm(-3000))
                                .onFalse(intakeSubsystem.stopRollerCommand());

                // Climb Sequences
                driverXbox.b()
                                .onTrue(drivebase.pathFindToPose(() -> drivebase.isRedAlliance()
                                                ? FieldConstants.kClimbPoseRed
                                                : FieldConstants.kClimbPoseBlue)
                                                .andThen(fullClimbSequence()))
                                .whileFalse(climbSubsystem.stopCommand());

                driverXbox.y().onTrue(
                                climbSubsystem.leadScrewHome().andThen(climbSubsystem.elevatorDown()));
        }

        private void configureEventTriggers() {
                new EventTrigger("Run_Intake")
                                .onTrue(intakeSubsystem.open())
                                .onFalse(intakeSubsystem.close());

                new EventTrigger("Bump")
                                .whileTrue(new InstantCommand(() -> passingBump = true))
                                .onFalse(new InstantCommand(() -> passingBump = false));

                new EventTrigger("SpinShooter")
                                .onTrue(Commands.runOnce(() -> shooterSubsystem.cmdEnableShooter(true),
                                                shooterSubsystem));

                new EventTrigger("Shoot")
                                .onTrue(generateShootCommand())
                                .onFalse(shooterSubsystem.cmdEnableShooter(false).andThen(hopperSubsystem.stopCommand()));

                new EventTrigger("ShootSim")
                                .onTrue(new InstantCommand(() -> ballSim.setShooting(true)))
                                .onFalse(new InstantCommand(() -> ballSim.setShooting(false)));
        }

        private void registerNamedCommands() {
                NamedCommands.registerCommand("Shoot_All", generateShootCommand());
                NamedCommands.registerCommand("Stop_Shooting",
                                shooterSubsystem.cmdEnableShooter(false).andThen(hopperSubsystem.stopCommand()));

                NamedCommands.registerCommand("Shoot_All_Sim",
                                new InstantCommand(() -> ballSim.setShooting(true))
                                                .andThen(new WaitCommand(5))
                                                .andThen(new InstantCommand(() -> ballSim.setShooting(false))));

                NamedCommands.registerCommand("Stop_Shooting_Sim",
                                new InstantCommand(() -> ballSim.setShooting(false)));

                NamedCommands.registerCommand("Climb_L1_Sim",
                                new InstantCommand(() -> climbed = !climbed));

                NamedCommands.registerCommand("Climb_L1", generateClimbL1Command());
        }

        // Command Generators
        private Command generateShootCommand() {
                return shooterSubsystem.cmdEnableShooter(true)
                                .andThen(new WaitUntilCommand(shooterSubsystem::atShooterSpeed))
                                .andThen(hopperSubsystem.push(0.5))
                                .andThen(new WaitUntilCommand(hopperSubsystem::isEmptyFor2s))
                                .andThen(shooterSubsystem.cmdEnableShooter(false))
                                .andThen(hopperSubsystem.stopCommand());
        }

        private Command generateClimbL1Command() {
                return climbSubsystem.leadScrewEngage()
                                .andThen(climbSubsystem.elevatorUp())
                                .andThen(climbSubsystem.elevatorDown());
        }

        private Command generateClimbL23Command() {
                return climbSubsystem.elevatorUp()
                                .andThen(climbSubsystem.elevatorDown());
        }

        private Command fullClimbSequence() {
                return generateClimbL1Command()
                                .andThen(generateClimbL23Command())
                                .andThen(generateClimbL23Command());
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
}