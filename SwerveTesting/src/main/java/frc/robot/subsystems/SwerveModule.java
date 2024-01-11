package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.SwerveDriveConstants;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANcoder angleEncoder;

  private String moduleName;

  private final SparkPIDController driveController;
  private final SparkPIDController angleController;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          SwerveDriveConstants.driveKS, SwerveDriveConstants.driveKV, SwerveDriveConstants.driveKA);

  public SwerveModule(int moduleNumber, String moduleName, SwerveModuleConstants constants) {
    this.moduleNumber = moduleNumber;
    this.moduleName = moduleName;
    this.angleOffset = constants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(constants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(constants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(constants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getRotations();
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(SwerveDriveConstants.angleContinuousCurrentLimit);
    angleMotor.setInverted(SwerveDriveConstants.angleInvert);
    angleMotor.setIdleMode(SwerveDriveConstants.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(SwerveDriveConstants.angleConversionFactor);
    angleController.setP(SwerveDriveConstants.angleKP);
    angleController.setI(SwerveDriveConstants.angleKI);
    angleController.setD(SwerveDriveConstants.angleKD);
    angleController.setFF(SwerveDriveConstants.angleKFF);
    angleMotor.enableVoltageCompensation(SwerveDriveConstants.voltageComp);
    angleMotor.burnFlash();
    Timer.delay(1);
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(SwerveDriveConstants.driveContinuousCurrentLimit);
    driveMotor.setInverted(SwerveDriveConstants.driveInvert);
    driveMotor.setIdleMode(SwerveDriveConstants.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(SwerveDriveConstants.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(SwerveDriveConstants.driveConversionPositionFactor);
    driveController.setP(SwerveDriveConstants.driveKP);
    driveController.setI(SwerveDriveConstants.driveKI);
    driveController.setD(SwerveDriveConstants.driveKD);
    driveController.setFF(SwerveDriveConstants.driveKFF);
    driveMotor.enableVoltageCompensation(SwerveDriveConstants.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / SwerveDriveConstants.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveDriveConstants.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  public void setAngleForX(double angle) {
    driveMotor.set(0);
    angleController.setReference(angle, ControlType.kPosition);
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public SwerveModulePosition getPosition() {
    SmartDashboard.putNumber("angleEncoder position " + moduleName, angleEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("angleOffset rotations " + moduleName, angleOffset.getRotations());

    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble() - angleOffset.getRotations()));
  }

  public String getSwerveModuleName() {
    return this.moduleName;
  }
}
