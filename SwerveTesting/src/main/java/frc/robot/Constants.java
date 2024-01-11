package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

  public static final class SwerveDriveConstants {
    public static final double fastDriveSpeedMultiplier = 1.0;
    public static final double normalDriveSpeedMultiplier = 0.6;
    public static final double slowDriveSpeedMultiplier = 0.2;

    public static final double stickDeadband = 0.1;

    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = 0.482598984;
    public static final double wheelBase = 0.4318000000000001714;
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio =
        (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // 6.75:1
    public static final double angleGearRatio = (150.0 / 7.0); //
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second  //4.5
    public static final double maxAngularVelocity = 6; // 11.5

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class FrontLeftSwerveModule {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      public static final String swerveModuleName = "FrontLeftSwerveModule";
    }

    /* Front Right Module - Module 1 */
    public static final class FrontRightSwerveModule {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        public static final String swerveModuleName = "FrontRightSwerveModule";
    }

    /* Back Left Module - Module 2 */
    public static final class BackLeftSwerveModule {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      public static final String swerveModuleName = "BackLeftSwerveModule";
    }

    /* Back Right Module - Module 3 */
    public static final class BackRightSwerveModule {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
      public static final String swerveModuleName = "BackRightSwerveModule";
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4; // 4
    public static final double kMaxAccelerationMetersPerSecondSquared = 3; // 3
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 2;
    public static final double kPYController = 2;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static HashMap<String, Command> namedEventMap = new HashMap<>();
    public static Alliance alliance;
  }

   public static final class VisionConstants {
    //15.25 inch h
    //19 3/4 v
    public static final String cameraName = "photonvision";
    public static final Transform3d robotToCam = new Transform3d(new Translation3d(.26, .2, 0), new Rotation3d()); 
    //public static final Transform3d ROBOT_TO_CAMERA = robotToCam.inverse();
    
  };

  public static final class FieldConstants {
    public static final double kLength = Units.feetToMeters(54);
    public static final double kWidth = Units.feetToMeters(27);
  }
  
}