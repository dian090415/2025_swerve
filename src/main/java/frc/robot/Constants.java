package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
	public static final class SwerveConstants {
		public static final double TRACK_WIDTH = 0.66; // 寬
		public static final double TRACK_LENGTH = 0.66; // 長
		public static final double WHEEL_RADIUS = 0.0508; // 輪子半徑
		
		public static final double MAX_SPEED = 3.0; // 最大速度m/s
		public static final double MAX_ACCLERATION = 1.0; // 最大加速度m/s^2
		public static final double MAX_ANGULAR_ACCLERATION = 9.0;
		public static final double DRIVE_GEAR_RATIO = 49.0 / 300.0; // 齒輪比
		public static final int MAX_VOLTAGE = 20;
		public static final double MAX_ANGULAR_ACCELERATION = 9.0;

		public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = WHEEL_RADIUS * 2 / DRIVE_GEAR_RATIO * Math.PI / 60;
		public static final double DRIVE_POSITION_CONVERSION_FACTOR = WHEEL_RADIUS * 2 / DRIVE_GEAR_RATIO * Math.PI;
	}

	public static final class MotorPort {
		public static final int FRONT_RIGHT = 0;
		public static final int FRONT_LEFT = 1;
		public static final int BACK_RIGHT = 2;
		public static final int BACK_LEFT = 3;
	}
	
	public static final class MotorReverse {
		public static final boolean FRONT_LEFT_DRIVE = true;
		public static final boolean FRONT_RIGHT_DRIVE = false;
		public static final boolean BACK_LEFT_DRIVE = true;
		public static final boolean BACK_RIGHT_DRIVE = false;

		public static final boolean FRONT_LEFT_TURN = true;
		public static final boolean FRONT_RIGHT_TURN = true;
		public static final boolean BACK_LEFT_TURN = true;
		public static final boolean BACK_RIGHT_TURN = true;
	}

	public static final class DriveEncoderReverse {
		public static final boolean FRONT_LEFT = true;
		public static final boolean FRONT_RIGHT = true;
		public static final boolean BACK_LEFT = true;
		public static final boolean BACK_RIGHT = true;
	}

	public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(Constants.SwerveConstants.TRACK_LENGTH / 2, Constants.SwerveConstants.TRACK_WIDTH / 2),
        new Translation2d(Constants.SwerveConstants.TRACK_LENGTH / 2, -Constants.SwerveConstants.TRACK_WIDTH / 2),
        new Translation2d(-Constants.SwerveConstants.TRACK_LENGTH / 2, Constants.SwerveConstants.TRACK_WIDTH / 2),
        new Translation2d(-Constants.SwerveConstants.TRACK_LENGTH / 2, -Constants.SwerveConstants.TRACK_WIDTH / 2)
    );

    public static final double DEAD_BAND = 0.05;
	public static final boolean gyroField = true;



	public static final String SwerveConstant = null;
}