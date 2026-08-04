package frc.robot;

import java.io.File;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public final class Constants {
    public static final class SwerveConstants {
        public static final String kSlash = "Swerve/";
        public static final File kSwerveConfigurationDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        // Theoretical no-load free speed: 21.97 ft/s
        // More realistic max speed: 16.81 ft/s
        public static final double kMaxSpeed = Units.feetToMeters(16);
    }

    public static final class IntakeConstants {
        public static final String kSlash = "Intake/";
        public static final int kIntakeMotorId = 13;
        public static final int kPivotMotorId = 14;

        public static final double kIntakeRPM = 5400;
        public static final double kIntakeRPMTolerance = 400;
        public static final double kPivotStallCurrentThreshold = 35;
    }

    public static final class ShooterConstants {
        public static final String kSlash = "Shooter/";
        public static final int kLowerIndexerMotorId = 15;
        public static final int kUpperIndexerMotorId = 16;
        public static final int kShooterLeftMotorId = 17;
        public static final int kShooterRightMotorId = 18;

        public static final double kShooterRPM = 2800;
        public static final double kShooterRPMTolerance = 200;
    }

    public static final class ClimberConstants {
        public static final String kSlash = "Climber/";
        public static final int kClimberMotorId = 19;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriverControllerDeadband = 0.1;
        public static final int kOperatorControllerPort = 1;
    }
}
