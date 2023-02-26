// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.autonomous.DoubleChargeSpin;
import frc.robot.commands.test.TestModuleCommand;
import frc.robot.commands.test.TestSwerveCommand;
import frc.robot.commands.test.TestSwerveRotationCommand;
import frc.robot.commands.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.autonomous.AutoUtils;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //Subsystems
  private SwerveDrivetrain m_drive;

  //Controllers
  private final CommandXboxController m_driveController = new CommandXboxController(Constants.DRIVER_PORT);

  //Logged chooser for auto
  private final LoggedDashboardChooser<Command> m_autoChooser = new LoggedDashboardChooser<>("Auto Modes");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      // Beta robot hardware implementation
      case THANOS:
      case HELIOS:
        m_drive = new SwerveDrivetrain();
        break;

      case SIM:
        break;

      // Default case, should be set to a replay mode
      default:
    }
    LiveWindow.disableAllTelemetry();

    // Configure the button bindings
    configureButtonBindings();
    configAutoChooser();
    configDashboard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_drive.setDefaultCommand(new SwerveTeleopDrive(m_drive, m_driveController));

    m_driveController.button(7).onTrue(m_drive.resetGyroBase());
    m_driveController.start().onTrue(m_drive.toggleFieldRelative());

  }

  /**
   * Use this method to add autonomous routines to a sendable chooser
   */
  public void configAutoChooser() {
    m_autoChooser.addDefaultOption("Forward Left", AutoUtils.getDefaultTrajectory(m_drive));
    m_autoChooser.addOption("Double Charge Spin", DoubleChargeSpin.getPath(m_drive).andThen(new AutoBalance(m_drive)));
    m_autoChooser.addOption("Test for Heading", DoubleChargeSpin.getTestPath(m_drive));
  }

  /**
   * This method sets up Shuffleboard tabs for test commands
   */
  public void configDashboard() {
    ShuffleboardTab testCommands = Shuffleboard.getTab("Commands");
    ShuffleboardTab testTrajectories = Shuffleboard.getTab("Trajectories");

    // Swerve Test Commands
    testCommands.add("Swerve Forward", new TestSwerveCommand(m_drive, 0));
    testCommands.add("Swerve Right", new TestSwerveCommand(m_drive, 90));
    testCommands.add("Swerve Backwards", new TestSwerveCommand(m_drive, 180));
    testCommands.add("Swerve Left", new TestSwerveCommand(m_drive, 270));

    testCommands.add("Swerve Clockwise", new TestSwerveRotationCommand(m_drive, false));
    testCommands.add("Swerve CounterClockwise", new TestSwerveRotationCommand(m_drive, true));

    testCommands.add("FL Module Test", new TestModuleCommand(m_drive, 0));
    testCommands.add("BL Module Test", new TestModuleCommand(m_drive, 2));
    
    testCommands.add("Auto Balance", new AutoBalance(m_drive));
    testCommands.add("Reset Pose", new InstantCommand(() -> m_drive.resetPoseBase())).withSize(2, 1);

    testTrajectories.add("Forward Traj", new InstantCommand(() -> {
      CommandScheduler.getInstance().schedule(
              AutoUtils.getDefaultTrajectory(m_drive)
      );
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.get();
  }
}
