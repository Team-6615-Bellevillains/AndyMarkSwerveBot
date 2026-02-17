package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveModule {

    private final WPI_TalonSRX driveMotor;
    private final WPI_TalonSRX steerMotor;

    //private final Encoder driveEncoder;

    //private final Encoder steerEncoder;

    private final ProfiledPIDController steerPIDController;

    private final double absoluteEncoderOffsetCounts;

    private final int idx;
    //private final double startingWheelRadians;

    public SwerveModule(int idx, int driverMotorID, int steerMotorID,
            boolean isDriveMotorReversed,
            double absoluteEncoderOffsetCounts) {

        this.idx = idx;

        this.absoluteEncoderOffsetCounts = absoluteEncoderOffsetCounts;

        this.driveMotor = new WPI_TalonSRX(driverMotorID);
        this.steerMotor = new WPI_TalonSRX(steerMotorID);

        // Cimcoder in TalonSRX
        this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        // Lamprey2 plugged into TalonSRX
        this.steerMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        this.steerMotor.configFeedbackNotContinuous(true, 0);

        /* 
        this.steerEncoder = new Encoder(steerEncoderAPort, steerEncoderBPort);
        this.steerEncoder.setDistancePerPulse(SwerveModuleConstants.kSteerEncoderRot2Rad);
        this.steerEncoder.reset();
        */
        
        //this.startingWheelRadians = getModuleRotationRadiansFromAbsoluteEncoder();

        this.driveMotor.setInverted(isDriveMotorReversed);
        

        this.steerPIDController = new ProfiledPIDController(SwerveModuleConstants.kPTurning,
                SwerveModuleConstants.kITurning,
                SwerveModuleConstants.kDTurning, new TrapezoidProfile.Constraints(
                        SwerveModuleConstants.maxWheelVelocity, SwerveModuleConstants.maxWheelAcceleration));
        this.steerPIDController.enableContinuousInput(0, 2 * Math.PI);

        //this.driveEncoder.setPosition(0);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDistance(),
                getModuleRotation());
    }

    public double getDistance() {
        return this.driveMotor.getSelectedSensorPosition();
    }

    public double getVelocity() {
        return driveMotor.getSelectedSensorVelocity();
    }

    private String appendIdx(String input) {
        return String.format("[%s] %s", idx, input);
    }

    public double getLampreyOutput() {
        return this.steerMotor.getSelectedSensorPosition();
    }

    private double getModuleRotationRadiansFromAbsoluteEncoder() {
        /*
         * Magnets that are read on absolute encoders are read as 0 on a random M_point
         * when assembled. We want this to be at the zero point of the wheels
         * (henceforth Z_point), so we must apply an offset.
         */
        double offsetCounts = this.steerMotor.getSelectedSensorPosition() - absoluteEncoderOffsetCounts;

        // Convert to radians using the formula ((amount/maxAmount) * 2 * π)
        return (offsetCounts / SwerveModuleConstants.maximumTotalCounts) * 2 * Math.PI;
    }

    public Rotation2d getModuleRotation() {
        /* 
        double encoderRadians = Math.IEEEremainder(steerEncoder.getDistance() + startingWheelRadians, 2 * Math.PI);

        // We don't like negative values, so we convert to the equivalent positive
        // angle.
        if (encoderRadians < 0) {
            encoderRadians += 2 * Math.PI;
        }

        return Rotation2d.fromRadians(encoderRadians);
        */
        return Rotation2d.fromRadians(this.getModuleRotationRadiansFromAbsoluteEncoder());
    }

    public void setDesiredState(SwerveModuleState state, boolean ignoreLittle) {
        if (ignoreLittle && Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state.optimize(getModuleRotation());

        double steerPIDOut = steerPIDController.calculate(getModuleRotation().getRadians(),
                state.angle.getRadians());

        double feedforward = SwerveSubsystem.calculateSteerFeedforward(steerPIDController.getSetpoint().velocity);
        double driveFF = SwerveSubsystem.calculateDriveFeedforward(state.speedMetersPerSecond);
        driveMotor.setVoltage(driveFF);
        steerMotor.setVoltage(steerPIDOut + feedforward);
        SmartDashboard.putNumber(appendIdx("drive voltage"), driveFF);
        SmartDashboard.putNumber(appendIdx("steer voltage"), steerPIDOut + feedforward);
        SmartDashboard.putNumber(appendIdx("measured position"), getModuleRotation().getRadians());
    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }

}
