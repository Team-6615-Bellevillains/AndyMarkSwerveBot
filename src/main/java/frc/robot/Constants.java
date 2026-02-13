package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
        
    public static final int kCimcoder256PulsesPerRevolution = 256;

    public static final class SwerveModuleConstants {
        public static final double kWheelCircumference = Units.inchesToMeters(4) * Math.PI;
        public static final double kDriveMotorGearRatio = 6.67 / 1;
        public static final double kSteerGearboxRatio = (71.0 / 1.0);
        public static final double kSteerModuleRatio = (48.0 / 40.0);
        public static final double kSteerPPR = 7;

        public static final double kPTurning = 8; // TODO: Tune, has not yet completely oscillated
        public static final double kITurning = 0;
        public static final double kDTurning = 0.0;

        public static final double kSTurning = 2.5466; // TODO: Tune with static test
        public static final double kVTurning = 0.97146;
        public static final double kATurning = 1.0961;

        public static final double maxWheelVelocity = 90.0/* revolutions/minute */ / 60.0/* seconds/minute */ * 2
                * Math.PI /* rads/revolution */;
        public static final double maxWheelAcceleration = maxWheelVelocity * 6;

        public static final double kDriveEncoderRot2Meter = kWheelCircumference / kDriveMotorGearRatio;
        public static final double kSteerEncoderRot2Rad = 2 * Math.PI
                / ((kSteerGearboxRatio / kSteerModuleRatio) * kSteerPPR);

        public static final int maximumTotalCounts = 1024;
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = 0.662; // Distance between right and left wheels
        public static final double kWheelBase = 0.504; // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 5;
        public static final int kFrontRightDriveMotorPort = 6;
        public static final int kBackLeftDriveMotorPort = 8;
        public static final int kBackRightDriveMotorPort = 7;

        public static final int kFrontLeftSteerMotorPort = 1;
        public static final int kFrontRightSteerMotorPort = 2;
        public static final int kBackLeftSteerMotorPort = 4;
        public static final int kBackRightSteerMotorPort = 3;

        /* 
        public static final int kFrontLeftEncoderAPort = 4;
        public static final int kFrontLeftEncoderBPort = 5;
        public static final int kFrontRightEncoderAPort = 6;
        public static final int kFrontRightEncoderBPort = 7;
        public static final int kBackLeftEncoderAPort = 2;
        public static final int kBackLeftEncoderBPort = 3;
        public static final int kBackRightEncoderAPort = 8;
        public static final int kBackRightEncoderBPort = 9;
        */

        public static final boolean kFrontLeftDriveMotorReversed = true;
        public static final boolean kFrontRightDriveMotorReversed = true;
        public static final boolean kBackLeftDriveMotorReversed = true;
        public static final boolean kBackRightDriveMotorReversed = true;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetCounts = 144;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetCounts = 784;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetCounts = 300;
        public static final double kBackRightDriveAbsoluteEncoderOffsetCounts = 503;

        /*
         * wheelPoint = (0,0) or (0,kWheelBase) or (kTrackWidth, 0) or (kTrackWidth,
         * kWheelBase)
         * centerPoint = (kTrackWidth/2, kWheelBase/2)
         * a = abs(centerPoint_x-wheelPoint_x) -> evaluates to kTrackWidth/2
         * b = abs(centerPoint_y-wheelPoint_y) -> evaluates to kWheelBase/2
         * kWheelDistanceFromCenter^2 = a^2 + b^2
         * kWheelDistanceFromCenter = sqrt(a^2+b^2)
         */
        public static final double kWheelDistanceFromCenter = Math
                .sqrt(Math.pow(kTrackWidth / 2, 2) + Math.pow(kWheelBase / 2, 2));

        public static final double kPhysicalMaxSpeedMetersPerSecond = Units.feetToMeters(13.5);

        /*
         * 2pi radians / (Circumference of the circle created by robot rotation aka the
         * distance travelled in one rotation / max speed)
         */
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI
                / (kWheelDistanceFromCenter / kPhysicalMaxSpeedMetersPerSecond);

        public static final double kTeleOpMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleOpMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
                / 35;
        public static final double kTeleOpMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleOpMaxAngularAccelerationUnitsPerSecond = 5;

        public static final double kPThetaCorrection = 5;
        public static final double kIThetaCorrection = 0;
        public static final double kDThetaCorrection = 0;
        public static final double kMaxVelocityThetaCorrection = 1;
        public static final double kMaxAccelerationThetaCorrection = 2;

        // TODO: Tune
        public static final double kPRotation = .25;
        public static final double kIRotation = 0;
        public static final double kDRotation = 0;

        // TODO: Tune
        public static final double kPMoveXDistance = 0;
        public static final double kIMoveXDistance = 0;
        public static final double kDMoveXDistance = 0;

        // TODO: Tune
        public static final double kPMoveYDistance = 0;
        public static final double kIMoveYDistance = 0;
        public static final double kDMoveYDistance = 0;

    }
}
