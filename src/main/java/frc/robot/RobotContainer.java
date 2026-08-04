@ -13,6 +13,7 @@ import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.UserConfig.DriveMode;
import frc.robot.subsystems.IntakeSubsystem;
@ -24,7 +25,7 @@ import frc.robot.util.Elastic.NotificationLevel;
import swervelib.SwerveInputStream;

public class RobotContainer {
  // Controllers
  // Controllers (both Xbox now)
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);
@ -34,6 +35,9 @@ public class RobotContainer {
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  // Tracks the currently active drive mode so it can be reapplied (e.g. after a controller reconnect)
  private DriveMode m_currentDriveMode = DriveMode.FieldOrientedAngularVelocity;

  // Swerve Input Streams
  private final SwerveInputStream m_robotRelative = SwerveInputStream.of(
      m_swerveSubsystem.getSwerveDrive(),
@ -65,6 +69,9 @@ public class RobotContainer {
    registerNamedCommands();
    configureBindings();

    // Apply the default drive mode at startup
    changeDriveMode(m_currentDriveMode);

    m_autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", m_autoChooser);
@ -80,23 +87,36 @@ public class RobotContainer {
  }

  private void configureBindings() {
    // --- Driver controller (Xbox) ---
    m_driverController.y().onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro()));

    m_driverController.leftTrigger(0.5)
        .whileTrue(m_intakeSubsystem.runIntake(0.5))
        .onFalse(m_intakeSubsystem.stopIntake());

    m_operatorController.button(7)
        .whileTrue(m_intakeSubsystem.runIntake(0.5))
        .onFalse(m_intakeSubsystem.stopIntake());

    m_operatorController.button(8).onTrue(m_intakeSubsystem.runIntake(1)).onFalse(m_intakeSubsystem.runIntake(0.5));

    m_driverController.rightTrigger(0.5)
        .whileTrue(m_shooterSubsystem.runShooter())
        .onFalse(m_shooterSubsystem.stopShooter());

    m_driverController.back().and(m_driverController.start()).onTrue(m_swerveSubsystem.zeroGyroWithAllianceCommand());
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
@ -113,6 +133,12 @@ public class RobotContainer {
    m_operatorController.a().onTrue(m_intakeSubsystem.setPivotPosition(0));

    m_operatorController.y().onTrue(m_intakeSubsystem.setPivotPosition(16));

    // Reapply the active drive mode automatically if the driver controller
    // connects/disconnects (e.g. swapped mid-match, battery died, etc.)
    new Trigger(() -> m_driverController.getHID().isConnected())
        .onTrue(new InstantCommand(() -> changeDriveMode(m_currentDriveMode)))
        .onFalse(new InstantCommand(() -> changeDriveMode(m_currentDriveMode)));
  }

  public void changeDriveMode(DriveMode driveMode) {
@ -120,6 +146,8 @@ public class RobotContainer {
      m_swerveSubsystem.getCurrentCommand().cancel();
    }

    m_currentDriveMode = driveMode;

    SwerveInputStream newInputStream = null;

    switch (driveMode) {
@ -131,7 +159,9 @@ public class RobotContainer {
        break;
      case FieldOrientedDirectAngle:
        newInputStream = m_allianceRelativeDirectAngle;
        break;
      default:
        newInputStream = m_allianceRelativeAngularVelocity;
        break;
    }

