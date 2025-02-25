package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.helpers.IDashboardProvider;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.SwerveSpark;
import edu.wpi.first.math.util.Units;

public class SwerveModule implements IDashboardProvider {
    private final SwerveSpark driveMotor;
    private final SwerveSpark turnMotor;

    private final RelativeEncoder driveEncoder;
    private final CANcoder turnEncoder;

    private final PIDController turnPid;

    private final boolean driveEncoderReversed;

    private final String motorName;
    private double driveOutput;
    private double turnOutput;

    public SwerveModule(int driveMotorPort, int turnMotorPort, int turnEncoderPort,
            boolean driveMotorReverse, boolean turnMotorReverse, boolean driveEncoderReverse,
            String motorName) {
        this.registerDashboard();

        this.driveMotor = new SwerveSpark(driveMotorPort, driveMotorReverse);
        this.turnMotor = new SwerveSpark(turnMotorPort, turnMotorReverse);

        this.driveEncoder = this.driveMotor.getEncoder();
        this.turnEncoder = new CANcoder(turnEncoderPort);

        this.driveEncoderReversed = driveEncoderReverse; // 是否反轉編碼器讀數

        this.turnPid = new PIDController(0.0065, 0.00005, 0.0);
        this.turnPid.enableContinuousInput(-180, 180); // 自動計算最短路徑到達設定點

        this.motorName = motorName;
    }

    public double getDriveEncoderPosition() {
        return this.driveEncoder.getPosition() * (this.driveEncoderReversed ? 1 : -1);
    }

    public double getDriveEncoderVelocity() {
        return this.driveEncoder.getVelocity() * (this.driveEncoderReversed ? 1 : -1);
    }

    public double getTurningEncoderPosition() {

        double value = Units.rotationsToDegrees(this.turnEncoder.getAbsolutePosition().getValueAsDouble());
        value %= 360.0;
        return value > 180 ? value - 360 : value;

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(this.getDriveEncoderVelocity(),
                Rotation2d.fromDegrees(this.getTurningEncoderPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(this.getDriveEncoderPosition(),
                Rotation2d.fromDegrees(this.getTurningEncoderPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            this.stop();
            return;
        }
        desiredState.optimize(this.getState().angle);//優化角度
        SwerveModuleState state = desiredState;

        this.driveOutput = state.speedMetersPerSecond / SwerveConstants.MAX_SPEED;
        this.turnOutput = this.turnPid.calculate(this.getState().angle.getDegrees(), state.angle.getDegrees());

        this.driveMotor.set(this.driveOutput);
        this.turnMotor.set(this.turnOutput);

    }

    public void stop() {
        this.driveMotor.stopMotor();
        this.turnMotor.stopMotor();
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putNumber(this.motorName + " Drive Vel", this.getState().speedMetersPerSecond);
        SmartDashboard.putNumber(this.motorName + " Turn Pos", this.getTurningEncoderPosition());
    }
}