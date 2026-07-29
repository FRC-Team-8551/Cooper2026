// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.UserConfig.DriveMode;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.NotificationLevel;
import swervelib.SwerveInputStream;

public class RobotContainer {
  // Controllers (both Xbox now)
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);

  // Subsystems
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // Tracks the currently active drive mode so it can be reapplied (e.g. after a controller reconnect)
  private DriveMode m_currentDriveMode = DriveMode.FieldOrientedAngularVelocity;

  // Swerve Input Streams
  private final SwerveInputStream m_robotRelative = SwerveInputStream.of(
      m_swerveSubsystem.getSwerveDrive(),
      () -> -m_driverController.getLeftY(),
      () -> -m_driverController.getLeftX())
      .withControllerRotationAxis(() -> -m_driverController.getRightX())
      .deadband(OIConstants.kDriverControllerDeadband)
      .scaleTranslation(0.8)
      .allianceRelativeControl(false);

  private final SwerveInputStream m_allianceRelativeAngularVelocity = m_robotRelative.copy()
      .allianceRelativeControl(true);

  private final SwerveInputStream m_allianceRelativeDirectAngle = m_allianceRelativeAngularVelocity.copy()
      .withControllerHeadingAxis(
          () -> m_driverController.getRightX() * (m_swerveSubsystem.isRedAlliance() ? 1 : -1),
          () -> m_driverController.getRightY() * (m_swerveSubsystem.isRedAlliance() ? 1 : -1))
      .headingWhile(true);

  // Commands

  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    // Interstellar reference
    Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Before you get all teary...",
        "Try to remember that as a robot, I have to do anything you say. Good luck, Cooper."));

    registerNamedCommands();
    configureBindings();

    // Apply the default drive mode at startup
    changeDriveMode(m_currentDriveMode);

    m_autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Run Intake", m_intakeSubsystem.runIntake(0.65));
    NamedCommands.registerCommand("Stop Intake", m_intakeSubsystem.stopIntake());
    NamedCommands.registerCommand("Run Shooter", m_shooterSubsystem.runShooter());
    NamedCommands.registerCommand("Stop Shooter", m_shooterSubsystem.stopShooter());
  }

  private void configureBindings() {
    // --- Driver controller (Xbox) ---
    m_driverController.y().onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro()));

    m_driverController.leftTrigger(0.5)
        .whileTrue(m_intakeSubsystem.runIntake(0.5))
        .onFalse(m_intakeSubsystem.stopIntake());

    m_driverController.rightTrigger(0.5)
        .whileTrue(m_shooterSubsystem.runShooter())
        .onFalse(m_shooterSubsystem.stopShooter());

    m_driverController.back().and(m_driverController.start())
        .onTrue(m_swerveSubsystem.zeroGyroWithAllianceCommand());

    // Example: toggle to direct-angle (heading) drive mode with X
    m_driverController.x()
        .onTrue(new InstantCommand(() -> changeDriveMode(DriveMode.FieldOrientedDirectAngle)));

    // Example: back to angular-velocity drive mode with B
    m_driverController.b()
        .onTrue(new InstantCommand(() -> changeDriveMode(DriveMode.FieldOrientedAngularVelocity)));

    // --- Operator controller (Xbox) ---
    m_operatorController.button(7)
        .whileTrue(m_intakeSubsystem.runIntake(0.5))
        .onFalse(m_intakeSubsystem.stopIntake());

    m_operatorController.button(8)
        .onTrue(m_intakeSubsystem.runIntake(1))
        .onFalse(m_intakeSubsystem.runIntake(0.5));

    m_operatorController.povUp().onTrue(m_intakeSubsystem.setIntakePivotSpeed(0.2))
        .onFalse(m_intakeSubsystem.setIntakePivotSpeed(0));

    m_operatorController.povDown().onTrue(m_intakeSubsystem.setIntakePivotSpeed(-0.2))
        .onFalse(m_intakeSubsystem.setIntakePivotSpeed(0));

    m_operatorController.button(3).whileTrue(m_intakeSubsystem.runIntake(-0.65))
        .onFalse(m_intakeSubsystem.stopIntake());

    m_operatorController.button(1).whileTrue(m_shooterSubsystem.reverseIndexers())
        .onFalse(m_shooterSubsystem.stopShooter());

    m_operatorController.a().onTrue(m_intakeSubsystem.setPivotPosition(0));

    m_operatorController.y().onTrue(m_intakeSubsystem.setPivotPosition(16));

    // Reapply the active drive mode automatically if the driver controller
    // connects/disconnects (e.g. swapped mid-match, battery died, etc.)
    new Trigger(() -> m_driverController.getHID().isConnected())
        .onTrue(new InstantCommand(() -> changeDriveMode(m_currentDriveMode)))
        .onFalse(new InstantCommand(() -> changeDriveMode(m_currentDriveMode)));
  }

  public void changeDriveMode(DriveMode driveMode) {
    if (m_swerveSubsystem.getCurrentCommand() != null) {
      m_swerveSubsystem.getCurrentCommand().cancel();
    }

    m_currentDriveMode = driveMode;

    SwerveInputStream newInputStream = null;

    switch (driveMode) {
      case RobotOriented:
        newInputStream = m_robotRelative;
        break;
      case FieldOrientedAngularVelocity:
        newInputStream = m_allianceRelativeAngularVelocity;
        break;
      case FieldOrientedDirectAngle:
        newInputStream = m_allianceRelativeDirectAngle;
        break;
      default:
        newInputStream = m_allianceRelativeAngularVelocity;
        break;
    }

    m_swerveSubsystem.setDefaultCommand(
        m_swerveSubsystem.drive(newInputStream, () -> m_driverController.axisGreaterThan(3, 0.5).getAsBoolean()));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
