package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.TunableSimpleMotorFeedforward;

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeft = new SwerveModule(
            0,
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftSteerMotorPort,
            DriveConstants.kFrontLeftDriveMotorReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetCounts);

    private final SwerveModule frontRight = new SwerveModule(
            1,
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightSteerMotorPort,
            DriveConstants.kFrontRightDriveMotorReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetCounts);

    private final SwerveModule backLeft = new SwerveModule(
            2,
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftSteerMotorPort,
            DriveConstants.kBackLeftDriveMotorReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetCounts);

    private final SwerveModule backRight = new SwerveModule(
            3,
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightSteerMotorPort,
            DriveConstants.kBackRightDriveMotorReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetCounts);

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            getRotation2d(), getModulePositions(), new Pose2d());

    private double lastKnownCorrectHeadingRadians;

    private final ProfiledPIDController thetaCorrectionPID = new ProfiledPIDController(DriveConstants.kPThetaCorrection,
            DriveConstants.kIThetaCorrection, DriveConstants.kDThetaCorrection, new TrapezoidProfile.Constraints(
                    DriveConstants.kMaxVelocityThetaCorrection, DriveConstants.kMaxAccelerationThetaCorrection));

    private static final TunableSimpleMotorFeedforward driveFeedforward = new TunableSimpleMotorFeedforward("drive",
            0.440000, 3.350000);

    private static final TunableSimpleMotorFeedforward steerFeedforward = new TunableSimpleMotorFeedforward("steer",
            1.112500, 1.300000);

    private double speedMultiplier = 2;

    private double gyroYawOffset = 0;

    public SwerveSubsystem() {
        this.thetaCorrectionPID.enableContinuousInput(0, 2 * Math.PI);

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();

                lastKnownCorrectHeadingRadians = getRotation2d().getRadians();
                this.thetaCorrectionPID.reset(lastKnownCorrectHeadingRadians);
            } catch (Exception e) {
                Commands.print("Failed to zero gyro on startup!");
            }
        }).start();
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
    }

    public void zeroHeading() {
        System.out.println("Zeroing gyro!");
        gyro.reset();
        gyroYawOffset = 0;
        System.out.println("Zeroed gyro!");
    }

    public Rotation2d getRotation2d() {
        return gyro.getRotation2d().rotateBy(Rotation2d.fromDegrees(gyroYawOffset));
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPoseEstimator(Pose2d pose) {
        gyroYawOffset = pose.getRotation().getDegrees();
        poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    public float getRawRollVelocity() {
        return gyro.getRawGyroY();
    }

    @Override
    public void periodic() {
        poseEstimator.update(getRotation2d(), getModulePositions());

        SmartDashboard.putNumber("Robot Heading", getRotation2d().getDegrees());
        SmartDashboard.putString("Robot Location",
                getPose().getTranslation().toString());

        SmartDashboard.putNumber("Last known correct heading Rads", lastKnownCorrectHeadingRadians);

        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Roll", gyro.getRoll());

        SmartDashboard.putNumber("Speed Multi", getSpeedMultiplier());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public void setSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = speedMultiplier;
    }

    public static double calculateDriveFeedforward(double velocity) {
        return driveFeedforward.getController().calculate(velocity);
    }

    public static double calculateSteerFeedforward(double velocity) {
        return steerFeedforward.getController().calculate(velocity);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates, boolean ignoreLittle) {
        // If some wheels are set to a speed over their max speed, they will cap out at
        // their max speed.
        // This messes up the ratio of the wheel speeds to each other.
        // This code will scale down the speeds so they are all within the max speed.
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0], ignoreLittle);
        frontRight.setDesiredState(desiredStates[1], ignoreLittle);
        backLeft.setDesiredState(desiredStates[2], ignoreLittle);
        backRight.setDesiredState(desiredStates[3], ignoreLittle);
    }

    public Command driveCommand(DoubleSupplier leftStickX, DoubleSupplier leftStickY, DoubleSupplier rightStickX){
        return this.run(()->{
            double fieldYSpeed = leftStickX.getAsDouble();
            double fieldXSpeed = leftStickY.getAsDouble();
            double angularSpeed = rightStickX.getAsDouble();
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldXSpeed, fieldYSpeed, angularSpeed, getRotation2d());
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            this.setModuleStates(moduleStates, false);
        });
    }
}
