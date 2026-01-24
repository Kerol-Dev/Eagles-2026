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

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.BallSim;
import swervelib.SwerveInputStream;

public class RobotContainer {

        private final CommandXboxController driverXbox = new CommandXboxController(0);

        public final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve"));

        private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
        private final HopperSubsystem hopperSubsystem = new HopperSubsystem();
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        private final LEDSubsystem ledSubsystem = new LEDSubsystem();
        public final BallSim ballSim = new BallSim();

        private SendableChooser<Command> autoChooser = new SendableChooser<>();

        private boolean m_intakeOpen = false;

        public static boolean climbed = false;
        public static boolean passingBump = false;

        private final SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
                        drivebase.getSwerveDrive(),
                        () -> -driverXbox.getLeftY(),
                        () -> -driverXbox.getLeftX())
                        .withControllerRotationAxis(driverXbox::getRightX)
                        .scaleRotation(-1)
                        .deadband(OperatorConstants.DEADBAND)
                        .allianceRelativeControl(true);

        public RobotContainer() {
                configureBindings();

                DriverStation.silenceJoystickConnectionWarning(true);

                new EventTrigger("Run_Intake").onTrue(intakeSubsystem.cmdOpen())
                                .onFalse(intakeSubsystem.cmdClose());

                new EventTrigger("Bump").whileTrue(new InstantCommand(() -> passingBump = true))
                                .onFalse(new InstantCommand(() -> passingBump = false));

                // NamedCommands.registerCommand("Shoot_All",
                // generateShootCommand());
                NamedCommands.registerCommand("Shoot_All",
                                new InstantCommand(() -> ballSim.setShooting(true)).andThen(new WaitCommand(5))
                                                .andThen(new InstantCommand(() -> ballSim.setShooting(false))));
                NamedCommands.registerCommand("Climb_L1",
                                new InstantCommand(() -> climbed = !climbed));
                NamedCommands.registerCommand("Climb_L1_Shoot",
                                generateClimbL1Command().andThen(generateShootCommand()));
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData(autoChooser);
        }

        private void configureBindings() {
                // Default behaviors
                drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity));
                hopperSubsystem.setDefaultCommand(hopperSubsystem.cmdIndexToSensor());
                shooterSubsystem.setDefaultCommand(
                                new RunCommand(() -> shooterSubsystem.aimAtTarget(drivebase.getPose(),
                                                drivebase.isRedAlliance()), shooterSubsystem));
                ledSubsystem.setDefaultCommand(
                                new RunCommand(() -> ledSubsystem.updateState(
                                                m_intakeOpen,
                                                shooterSubsystem.lockedAtTarget(),
                                                shooterSubsystem.atShooterSpeed()),
                                                ledSubsystem));

                driverXbox.start().onTrue(Commands.runOnce(drivebase::zeroGyro));

                // Shoot: spin up -> wait for speed -> feed
                driverXbox.rightTrigger()
                                .whileTrue(
                                                shooterSubsystem.cmdEnableShooter(true)
                                                                .andThen(new WaitUntilCommand(
                                                                                () -> shooterSubsystem
                                                                                                .atShooterSpeed()))
                                                                .andThen(hopperSubsystem.cmdPush(0.25)))
                                .onFalse(shooterSubsystem.cmdEnableShooter(false).andThen(hopperSubsystem.cmdStop()));
                // Intake open/close toggle (flip boolean first, then choose)
                driverXbox.rightBumper().onTrue(
                                Commands.runOnce(() -> m_intakeOpen = !m_intakeOpen)
                                                .andThen(Commands.either(
                                                                intakeSubsystem.cmdOpen(),
                                                                intakeSubsystem.cmdClose(),
                                                                () -> m_intakeOpen)));

                // Intake rollers
                driverXbox.leftTrigger()
                                .whileTrue(intakeSubsystem.cmdRunRollerRpm(-3000))
                                .onFalse(intakeSubsystem.cmdStopRoller());

                // Climb
                driverXbox.b().onTrue(generateClimbL3Command());
                driverXbox.a().whileTrue(climbSubsystem.cmdUp(0.5)).onFalse(climbSubsystem.cmdStop());
                driverXbox.y().whileTrue(climbSubsystem.cmdDown(0.5)).onFalse(climbSubsystem.cmdStop());
        }

        private Command generateShootCommand() {
                return shooterSubsystem.cmdEnableShooter(true)
                                .andThen(new WaitUntilCommand(() -> shooterSubsystem.atShooterSpeed()))
                                .andThen(hopperSubsystem.cmdPush(0.25))
                                .andThen(new WaitUntilCommand(() -> hopperSubsystem.isEmptyFor2s()))
                                .andThen(shooterSubsystem.cmdEnableShooter(false))
                                .andThen(hopperSubsystem.cmdStop());
        }

        private Command generateClimbL1Command() {
                return climbSubsystem.cmdPushGoToPosition(20)
                                .andThen(new WaitUntilCommand(() -> climbSubsystem.atPushPosition(20)))
                                .andThen(climbSubsystem.cmdGoToPosition(200))
                                                .andThen(new WaitUntilCommand(
                                                                () -> climbSubsystem.atElevatorPosition(200)))
                                                .andThen(climbSubsystem.cmdGoToPosition(0))
                                                .andThen(new WaitUntilCommand(
                                                                () -> climbSubsystem.atElevatorPosition(0)));
        }

        private Command buildClimbSingleCycle() {
                return climbSubsystem.cmdGoToPosition(200)
                                .andThen(new WaitUntilCommand(() -> climbSubsystem.atElevatorPosition(200)))
                                .andThen(climbSubsystem.cmdGoToPosition(0))
                                .andThen(new WaitUntilCommand(() -> climbSubsystem.atElevatorPosition(0)));
        }

        private Command generateClimbL3Command() {
                return Commands.sequence(
                                generateClimbL1Command(),
                                buildClimbSingleCycle(),
                                buildClimbSingleCycle());
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void setMotorBrake(boolean brake) {
                drivebase.setMotorBrake(brake);
        }
}