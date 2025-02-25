package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.SwerveConstants;

public class Driver extends XboxController {
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(SwerveConstants.MAX_ACCLERATION);//控制機器人前進/側移的加速度（線性速度變化率）
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(SwerveConstants.MAX_ANGULAR_ACCLERATION);//控制機器人的旋轉加速度（角速度變化率）
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveConstants.MAX_ANGULAR_ACCELERATION);//限制角加速度

    public Driver() {
        super(0);
    }

    public double getXDesiredSpeed() {
        double speed = -MathUtil.applyDeadband(this.getLeftY(), Constants.DEAD_BAND);
        return this.xLimiter.calculate(speed);
    }

    public double getYDesiredSpeed() {
        double speed = -MathUtil.applyDeadband(this.getLeftX(), Constants.DEAD_BAND);
        return this.yLimiter.calculate(speed);
    }

    public double getRotationSpeed() {
        double speed = -MathUtil.applyDeadband(this.getRightX(), Constants.DEAD_BAND);
        return this.rotationLimiter.calculate(speed);
    }
}