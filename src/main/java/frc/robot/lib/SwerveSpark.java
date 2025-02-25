package frc.robot.lib;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.SwerveConstants;

public class SwerveSpark extends SparkMax {
    public SwerveSpark(int motorport, boolean reverse) {
        super(motorport, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(reverse)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(SwerveConstants.MAX_VOLTAGE);
        config.encoder
                .positionConversionFactor(SwerveConstants.DRIVE_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(SwerveConstants.DRIVE_VELOCITY_CONVERSION_FACTOR);
        this.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
