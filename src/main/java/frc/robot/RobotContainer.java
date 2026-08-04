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
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
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
  // Both an Xbox and a PS5 wrapper per seat, same port -- whichever pad isn't
  // plugged in just reads as "nothing pressed," so OR-ing bindings across both is safe.
  private final CommandXboxController m_driverXbox = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandPS5Controller m_driverPS5 = new CommandPS5Controller(OIConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorXbox = new CommandXboxController(OIConstants.kOperatorControllerPort);
  private final CommandPS5Controller m_operatorPS5 = new CommandPS5Controller(OIConstants.kOperatorControllerPort);

  // Subsystems
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // Tracks the currently active drive mode so it can be reapplied (e.g. after a controller reconnect)
  private DriveMode m_currentDriveMode = DriveMode.FieldOrientedAngularVelocity;

  // Stick helpers: ask both wrappers, use whichever is actually moving
  private double driverLeftY() {
    double xbox = -m_driverXbox.getLeftY();
    return xbox != 0.0 ? xbox : -m_driverPS5.getLeftY();
  }

  private double driverLeftX() {
    double xbox = -m_driverXbox.getLeftX();
    return xbox != 0.0 ? xbox : -m_driverPS5.getLeftX();
  }

  private double driverRightX() {
    double xbox = -m_driverXbox.getRightX();
    return xbox != 0.0 ? xbox : -m_driverPS5.getRightX();
  }

  private double driverRightY() {
    double xbox = m_driverXbox.getRightY();
    return xbox != 0.0 ? xbox : m_driverPS5.getRightY();
  }

  // Swerve Input Streams
  private final SwerveInputStream m_robotRelative = SwerveInputStream.of(
      m_swerveSubsystem.getSwerveDrive(),
      this::driverLeftY,
      this::driverLeftX)
      .withControllerRotationAxis(this::driverRightX)
      .deadband(OIConstants.kDriverControllerDeadband)
      .scaleTranslation(0.8)
      .allianceRelativeControl(false);

  private final SwerveInputStream m_allianceRelativeAngularVelocity = m_robotRelative.copy()
      .allianceRelativeControl(true);

  private final SwerveInputStream m_allianceRelativeDirectAngle = m_allianceRelativeAngularVelocity.copy()
      .withControllerHeadingAxis(
          () -> driverRightX() * (m_swerveSubsystem.isRedAlliance() ? 1 : -1),
          () -> driverRightY() * (m_swerveSubsystem.isRedAlliance() ? 1 : -1))
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
    // --- Driver controller ---
    Trigger driverZeroGyro = m_driverXbox.y().or(m_driverPS5.triangle());
    driverZeroGyro.onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro()));

    Trigger driverIntake = m_driverXbox.leftTrigger(0.5).or(m_driverPS5.L2());
    driverIntake
        .whileTrue(m_intakeSubsystem.runIntake(0.5))
        .onFalse(m_intakeSubsystem.stopIntake());

    Trigger driverShoot = m_driverXbox.rightTrigger(0.5).or(m_driverPS5.R2());
    driverShoot
        .whileTrue(m_shooterSubsystem.runShooter())
        .onFalse(m_shooterSubsystem.stopShooter());

    // PS5 renamed "Share" to "Create" -- PS5Controller has no share() method
    Trigger driverZeroGyroAlliance = m_driverXbox.back().and(m_driverXbox.start())
        .or(m_driverPS5.create().and(m_driverPS5.options()));
    driverZeroGyroAlliance.onTrue(m_swerveSubsystem.zeroGyroWithAllianceCommand());

    Trigger driverDirectAngle = m_driverXbox.x().or(m_driverPS5.square());
    driverDirectAngle
        .onTrue(new InstantCommand(() -> changeDriveMode(DriveMode.FieldOrientedDirectAngle)));

    Trigger driverAngularVelocity = m_driverXbox.b().or(m_driverPS5.circle());
    driverAngularVelocity
        .onTrue(new InstantCommand(() -> changeDriveMode(DriveMode.FieldOrientedAngularVelocity)));

    // --- Operator controller ---
    // Using named methods here instead of raw button(N) -- the index numbers
    // don't line up between Xbox and PS5 pads (e.g. index 7 is Back on Xbox but L2 on PS5)
    Trigger opIntakeHold = m_operatorXbox.back().or(m_operatorPS5.create());
    opIntakeHold
        .whileTrue(m_intakeSubsystem.runIntake(0.5))
        .onFalse(m_intakeSubsystem.stopIntake());

    Trigger opIntakeFull = m_operatorXbox.start().or(m_operatorPS5.options());
    opIntakeFull
        .onTrue(m_intakeSubsystem.runIntake(1))
        .onFalse(m_intakeSubsystem.runIntake(0.5));

    Trigger opPivotUp = m_operatorXbox.povUp().or(m_operatorPS5.povUp());
    opPivotUp.onTrue(m_intakeSubsystem.setIntakePivotSpeed(0.2))
        .onFalse(m_intakeSubsystem.setIntakePivotSpeed(0));

    Trigger opPivotDown = m_operatorXbox.povDown().or(m_operatorPS5.povDown());
    opPivotDown.onTrue(m_intakeSubsystem.setIntakePivotSpeed(-0.2))
        .onFalse(m_intakeSubsystem.setIntakePivotSpeed(0));

    Trigger opReverseIntake = m_operatorXbox.x().or(m_operatorPS5.square());
    opReverseIntake.whileTrue(m_intakeSubsystem.runIntake(-0.65))
        .onFalse(m_intakeSubsystem.stopIntake());

    // Reverse indexers: A on Xbox, Cross on PS5
    Trigger opReverseIndexers = m_operatorXbox.a().or(m_operatorPS5.cross());
    opReverseIndexers.whileTrue(m_shooterSubsystem.reverseIndexers())
        .onFalse(m_shooterSubsystem.stopShooter());

    // Pivot to 0: B on Xbox, Circle on PS5
    Trigger opPivotZero = m_operatorXbox.b().or(m_operatorPS5.circle());
    opPivotZero.onTrue(m_intakeSubsystem.setPivotPosition(0));

    // Pivot to 16: Y on Xbox, Triangle on PS5
    Trigger opPivotSixteen = m_operatorXbox.y().or(m_operatorPS5.triangle());
    opPivotSixteen.onTrue(m_intakeSubsystem.setPivotPosition(16));

    // Covers a battery dying or pad swap mid-match
    Trigger driverConnectionChanged = new Trigger(
        () -> m_driverXbox.getHID().isConnected() || m_driverPS5.getHID().isConnected());
    driverConnectionChanged
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

    // Right trigger axis index differs: Xbox = 3, PS5 R2 = 4
    m_swerveSubsystem.setDefaultCommand(
        m_swerveSubsystem.drive(newInputStream, () -> m_driverXbox.axisGreaterThan(3, 0.5).getAsBoolean()
            || m_driverPS5.axisGreaterThan(4, 0.5).getAsBoolean()));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}