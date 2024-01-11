package frc.lib.config;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;

public final class CTREConfigs {
  public MagnetSensorConfigs swerveCanCoderConfig;

  public CTREConfigs() {
    swerveCanCoderConfig = new MagnetSensorConfigs();

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    swerveCanCoderConfig.SensorDirection = Constants.SwerveDriveConstants.canCoderInvert ? SensorDirectionValue.CounterClockwise_Positive :  SensorDirectionValue.Clockwise_Positive;
  }
}
