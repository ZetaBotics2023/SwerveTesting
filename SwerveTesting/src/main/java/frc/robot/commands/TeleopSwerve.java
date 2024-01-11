package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private DriveSubsystem m_driveSubsystem;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier robotCentricSup;
  private BooleanSupplier slowSpeedSup;
  private BooleanSupplier highSpeedSup;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  public TeleopSwerve(
      DriveSubsystem m_driveSubsystem,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier robotCentricSup,
      BooleanSupplier slowSpeedSup,
      BooleanSupplier highSpeedSup) {
    this.m_driveSubsystem = m_driveSubsystem;
    addRequirements(m_driveSubsystem);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.robotCentricSup = robotCentricSup;
    this.slowSpeedSup = slowSpeedSup;
    this.highSpeedSup = highSpeedSup;
  }

  @Override
  public void execute() {

    double speedMultiplier = SwerveDriveConstants.normalDriveSpeedMultiplier;
    if (highSpeedSup.getAsBoolean()) speedMultiplier = SwerveDriveConstants.fastDriveSpeedMultiplier;
    if (slowSpeedSup.getAsBoolean()) speedMultiplier = SwerveDriveConstants.slowDriveSpeedMultiplier;

    /* Get Values, Deadband*/
    double translationVal =
        translationLimiter.calculate(
            speedMultiplier
                * MathUtil.applyDeadband(
                    translationSup.getAsDouble(), SwerveDriveConstants.stickDeadband));
    double strafeVal =
        strafeLimiter.calculate(
            speedMultiplier
                * MathUtil.applyDeadband(strafeSup.getAsDouble(), SwerveDriveConstants.stickDeadband));
    double rotationVal =
        rotationLimiter.calculate(
            speedMultiplier
                * MathUtil.applyDeadband(
                    rotationSup.getAsDouble(), SwerveDriveConstants.stickDeadband));

    /* Drive */
    m_driveSubsystem.drive(
        new Translation2d(translationVal, strafeVal).times(SwerveDriveConstants.maxSpeed),
        rotationVal * SwerveDriveConstants.maxAngularVelocity,
        !robotCentricSup.getAsBoolean(),
        false);
  }
}
