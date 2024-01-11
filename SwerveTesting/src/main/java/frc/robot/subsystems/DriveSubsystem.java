package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveDriveConstants.BackLeftSwerveModule;
import frc.robot.Constants.SwerveDriveConstants.BackRightSwerveModule;
import frc.robot.Constants.SwerveDriveConstants.FrontLeftSwerveModule;
import frc.robot.Constants.SwerveDriveConstants.FrontRightSwerveModule;
import frc.robot.Constants.VisionConstants;

public class DriveSubsystem extends SubsystemBase {
  // private final ADIS16470_IMU gyro;
  private final Pigeon2 gyro;

  private SwerveModule[] mSwerveMods;

  private final PoseEstimatorSubsystem m_poseEstimatorSubsystem;

  private Field2d field;

  public DriveSubsystem() {
    // gyro = new ADIS16470_IMU();
    gyro = new Pigeon2(13);
    // gyro.configFactoryDefault();
    zeroGyro();

    this.m_poseEstimatorSubsystem = new PoseEstimatorSubsystem(new PhotonCamera(VisionConstants.cameraName), this);

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, FrontLeftSwerveModule.swerveModuleName, FrontLeftSwerveModule.constants),
          new SwerveModule(1, FrontRightSwerveModule.swerveModuleName, FrontRightSwerveModule.constants),
          new SwerveModule(2, BackLeftSwerveModule.swerveModuleName, BackLeftSwerveModule.constants),
          new SwerveModule(3, BackRightSwerveModule.swerveModuleName, BackRightSwerveModule.constants)
        };

    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        SwerveDriveConstants.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveDriveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public PoseEstimatorSubsystem getPoseEstimatorSubsystem() {
    return this.m_poseEstimatorSubsystem;
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveDriveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false); // false
    }
    SmartDashboard.putNumber("FL Desired MPS", desiredStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("FL Desired Angle", desiredStates[0].angle.getDegrees());

    SmartDashboard.putNumber("FR Desired MPS", desiredStates[1].speedMetersPerSecond);
    SmartDashboard.putNumber("FR Desired Angle", desiredStates[1].angle.getDegrees());

    SmartDashboard.putNumber("BL Desired MPS", desiredStates[2].speedMetersPerSecond);
    SmartDashboard.putNumber("BL Desired Angle", desiredStates[2].angle.getDegrees());

    SmartDashboard.putNumber("BR Desired MPS", desiredStates[3].speedMetersPerSecond);
    SmartDashboard.putNumber("BR Desired Angle", desiredStates[3].angle.getDegrees());
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "position: module " + mod.moduleNumber, mod.getPosition().distanceMeters);
      SmartDashboard.putNumber(
          "angle: module " + mod.moduleNumber, mod.getPosition().angle.getDegrees());
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.reset();
  }

  public Rotation2d getYaw() {
    return (SwerveDriveConstants.invertGyro)
        ? Rotation2d.fromDegrees(gyro.getAngle())
        : Rotation2d.fromDegrees(360 - gyro.getAngle());
  }

  // public double getXFilteredAccelAngle() {
  //   return gyro.getXFilteredAccelAngle();
  // }

  // public double getYFilteredAccelAngle() {
  //   return gyro.getYFilteredAccelAngle();
  // }


  // SmartDashboard.putNumber("gyro filtered X", gyro.getXFilteredAccelAngle()); // loops between
  // about 14...0...360...346
  // SmartDashboard.putNumber("gyro filtered Y", gyro.getYFilteredAccelAngle()); // forward and back
  // leveling
  // 0-14, drive forward, 346-360 drive backward

  public void setX() {
    mSwerveMods[0].setAngleForX(45);
    mSwerveMods[1].setAngleForX(-45);
    mSwerveMods[2].setAngleForX(-45);
    mSwerveMods[3].setAngleForX(45);
  }

  @Override
  public void periodic() {    
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.getSwerveModuleName() + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.getSwerveModuleName() + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.getSwerveModuleName() + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
