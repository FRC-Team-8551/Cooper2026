// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
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
  // Controllers
  private final CommandPS5Controller m_driverController = new CommandPS5Controller(OIConstants.kDriverControllerPort);
  private final CommandPS5Controller m_operatorController = new CommandPS5Controller(
      OIConstants.kOperatorControllerPort);

  // Subsystems
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // Swerve Input Streams
  // NOTE: getLeftX/getLeftY/getRightX are read continuously every loop, so
  // translation and rotation already return to zero (and the robot stops
  // moving/turning) the instant a stick is released — no extra binding needed.
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
    // Xbox Y -> PS5 Triangle
    m_driverController.triangle().onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro()));

    // Xbox LT axis-trigger -> PS5 L2 axis-trigger
    m_driverController.axisGreaterThan(PS5Controller.Axis.kL2.value, 0.5)
        .whileTrue(m_intakeSubsystem.runIntake(0.5))
        .onFalse(m_intakeSubsystem.stopIntake());

    // Xbox button(7)=Back -> PS5 Create
    m_operatorController.create()
        .whileTrue(m_intakeSubsystem.runIntake(0.5))
        .onFalse(m_intakeSubsystem.stopIntake());

    // Xbox button(8)=Start -> PS5 Options
    m_operatorController.options().onTrue(m_intakeSubsystem.runIntake(1)).onFalse(m_intakeSubsystem.runIntake(0.5));

    // Xbox RT axis-trigger -> PS5 R2 axis-trigger
    m_driverController.axisGreaterThan(PS5Controller.Axis.kR2.value, 0.5)
        .whileTrue(m_shooterSubsystem.runShooter())
        .onFalse(m_shooterSubsystem.stopShooter());

    // Xbox Back+Start -> PS5 Create+Options
    m_driverController.create().and(m_driverController.options()).onTrue(m_swerveSubsystem.zeroGyroWithAllianceCommand());

    m_operatorController.povUp().onTrue(m_intakeSubsystem.setIntakePivotSpeed(0.2))
        .onFalse(m_intakeSubsystem.setIntakePivotSpeed(0));

    m_operatorController.povDown().onTrue(m_intakeSubsystem.setIntakePivotSpeed(-0.2))
        .onFalse(m_intakeSubsystem.setIntakePivotSpeed(0));

    // Xbox X -> PS5 Square
    m_operatorController.square().whileTrue(m_intakeSubsystem.runIntake(-0.65))
        .onFalse(m_intakeSubsystem.stopIntake());

    // Xbox A(button 1) -> PS5 Cross
    m_operatorController.cross().whileTrue(m_shooterSubsystem.reverseIndexers())
        .onFalse(m_shooterSubsystem.stopShooter());

    // Was previously ALSO bound to A (a()) in the Xbox version -> double bind.
    // Moved to Circle, which was unused, so Cross and Circle each do one thing.
    m_operatorController.circle().onTrue(m_intakeSubsystem.setPivotPosition(0));

    // Xbox Y -> PS5 Triangle
    m_operatorController.triangle().onTrue(m_intakeSubsystem.setPivotPosition(16));
  }

  public void changeDriveMode(DriveMode driveMode) {
    if (m_swerveSubsystem.getCurrentCommand() != null) {
      m_swerveSubsystem.getCurrentCommand().cancel();
    }

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
        break; // was missing — fell through into default before
      default:
        break;
    }

    // PS5 axis order differs from Xbox: R2 is axis index 4, not 3
    // (Xbox: kRightTrigger=3 | PS5: kLeftX=0,kLeftY=1,kRightX=2,kL2=3,kR2=4,kRightY=5)
    m_swerveSubsystem.setDefaultCommand(
        m_swerveSubsystem.drive(newInputStream,
            () -> m_driverController.axisGreaterThan(PS5Controller.Axis.kR2.value, 0.5).getAsBoolean()));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}