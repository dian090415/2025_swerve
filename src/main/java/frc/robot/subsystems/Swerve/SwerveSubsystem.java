package frc.robot.subsystems.Swerve;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DeviceId.Neo;
import frc.robot.lib.helpers.IDashboardProvider;
import frc.robot.DeviceId.Encoder;
import frc.robot.Constants.MotorReverse;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.DriveEncoderReverse;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase implements IDashboardProvider {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final AHRS gyro;
    private final SwerveDriveOdometry odometry;
    private final StructPublisher<Pose3d> posePublisher;
    private final StructArrayPublisher<Pose3d> poseArrayPublisher;

    public SwerveSubsystem() {
        this.posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("MyPose", Pose3d.struct).publish();
        this.poseArrayPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();

        this.registerDashboard();
        this.frontLeft = new SwerveModule(Neo.frontLeftDrive, Neo.frontLeftTurn, Encoder.frontLeft,
                MotorReverse.FRONT_LEFT_DRIVE, MotorReverse.FRONT_LEFT_TURN, DriveEncoderReverse.FRONT_LEFT,
                "frontLeft");
        this.frontRight = new SwerveModule(Neo.frontRightDrive, Neo.frontRightTurn, Encoder.frontRight,
                MotorReverse.FRONT_RIGHT_DRIVE, MotorReverse.FRONT_RIGHT_TURN, DriveEncoderReverse.FRONT_RIGHT,
                "frontRight");
        this.backLeft = new SwerveModule(Neo.backLeftDrive, Neo.backLeftTurn, Encoder.backLeft,
                MotorReverse.BACK_LEFT_DRIVE, MotorReverse.BACK_LEFT_TURN, DriveEncoderReverse.BACK_LEFT, "backLeft");
        this.backRight = new SwerveModule(Neo.backRightDrive, Neo.backRightTurn, Encoder.backRight,
                MotorReverse.BACK_RIGHT_DRIVE, MotorReverse.BACK_RIGHT_TURN, DriveEncoderReverse.BACK_RIGHT,
                "backRight");
        this.gyro = new AHRS(NavXComType.kMXP_SPI);
        this.odometry = new SwerveDriveOdometry(
                Constants.swerveDriveKinematics, this.gyro.getRotation2d(), this.getModulePosition());
        this.gyro.reset();
    }

    @Override
    public void periodic() {
        this.odometry.update(this.gyro.getRotation2d(), getModulePosition());
        // 取得當前 Pose2d (2D 位置和旋轉)

        
        Pose2d pose2d = this.getPose();

        // 把 Pose2d 轉換成 Pose3d (假設機器人 z 軸高度是 0)
        Pose3d pose3d = new Pose3d(
                new Translation3d(pose2d.getX(), pose2d.getY(), 0.0),
                new Rotation3d(0.0, 0.0, pose2d.getRotation().getRadians()));

        // 發送單一 Pose3d
        this.posePublisher.set(pose3d);

        // 發送 Pose3d 陣列 (目前只包含一個)
        this.poseArrayPublisher.set(new Pose3d[] { pose3d });
    }

    public void resetGyro() {
        this.gyro.reset();
    }

    public void driveSwerve(double xSpeed, double ySpeed, double rotation, boolean field) {
        SwerveModuleState[] state = Constants.swerveDriveKinematics.toSwerveModuleStates(
                field ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, this.gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rotation));
        this.setModuleState(state);
    }

    public void setModuleState(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_SPEED);
        this.frontLeft.setDesiredState(states[0]);
        this.frontRight.setDesiredState(states[1]);
        this.backLeft.setDesiredState(states[2]);
        this.backRight.setDesiredState(states[3]);
    }

    public SwerveModuleState[] getModuleState() {
        return new SwerveModuleState[] {
                this.frontLeft.getState(),
                this.frontRight.getState(),
                this.backLeft.getState(),
                this.backRight.getState()
        };
    }

    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[] {
                this.frontLeft.getPosition(),
                this.frontRight.getPosition(),
                this.backLeft.getPosition(),
                this.backRight.getPosition()
        };
    }

    public void wait(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public Pose2d getPose() {
        return this.odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        this.odometry.resetPosition(this.gyro.getRotation2d(), this.getModulePosition(), pose);
    }

    public ChassisSpeeds getSpeeds() {
        return Constants.swerveDriveKinematics.toChassisSpeeds(this.getModuleState());
    }

    public void stopModules() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }

    @Override
    public void putDashboard() {
    }
}