package frc.robot.subsystems.Swerve;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {
    private SparkMax driveMotor, rotationMotor;
    private RelativeEncoder driveEncoder, rotationEncoder;
    private SparkClosedLoopController rotationPID;
    private PIDController drivePID;

    public SwerveModule(int driveMotorId, int RotationMotorId, boolean DriveMotorReversed, boolean RotationMotorReversed) {
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        rotationMotor = new SparkMax(RotationMotorId, MotorType.kBrushless);
        this.driveEncoder = driveMotor.getEncoder();
        this.rotationEncoder = rotationMotor.getEncoder();
        this.rotationPID = rotationMotor.getClosedLoopController();
        drivePID = new PIDController(0.3, 0.0, 0.0);

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.smartCurrentLimit(80);
        driveConfig.encoder.positionConversionFactor(Constants.ModuleConstants.DriveEncoderPositionConversionFactor);
        driveConfig.encoder.velocityConversionFactor(Constants.ModuleConstants.DriveEncoderVelocityConversionFactor);
        driveConfig.inverted(DriveMotorReversed);
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        SparkMaxConfig rotationConfig = new SparkMaxConfig();
        rotationConfig.idleMode(IdleMode.kCoast);
        rotationConfig.smartCurrentLimit(80);
        rotationConfig.encoder.positionConversionFactor(Constants.ModuleConstants.RotationEncoderPositionConversionFactor);
        rotationConfig.encoder.velocityConversionFactor(Constants.ModuleConstants.RotationEncoderVelocityConversionFactor);
        rotationConfig.inverted(RotationMotorReversed);
        rotationConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            Constants.ModuleConstants.kP, 
            Constants.ModuleConstants.kI, 
            Constants.ModuleConstants.kD
        )
        .outputRange(-1, 1)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-180, 180); 
        rotationMotor.configure(rotationConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        resetEncoders();
    }

    public double GetRotationPosition() {
        return rotationEncoder.getPosition();
    }

    public double GetRotationVelocity() {
        return rotationEncoder.getVelocity();
    }

    public double GetDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double GetDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public SwerveModulePosition GetModulePosition() {
        return new SwerveModulePosition(GetDrivePosition(), Rotation2d.fromDegrees(GetRotationPosition()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(GetDriveVelocity(), Rotation2d.fromDegrees(GetRotationPosition()));
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        rotationEncoder.setPosition(0);
    }

    public void setDesiredState(SwerveModuleState state) {
        if(Math.abs(state.speedMetersPerSecond) < 0.02) {
            stop(); 
            return;
        }
        
        //osmanın kırk saniyede uyuduğu fonksiyon
        state.optimize(Rotation2d.fromDegrees(GetRotationPosition()));

        driveMotor.set(state.speedMetersPerSecond / Constants.ModuleConstants.PhysicalMaxSpeed);


        rotationPID.setSetpoint(state.angle.getDegrees(), ControlType.kPosition);
    }

    //FIXME kontrol edilecek
    public void setDesiredStateFeedForward(SwerveModuleState state, double feedforwardVolts) {
        if(Math.abs(state.speedMetersPerSecond) < 0.02) {
            stop(); 
            return;
        }
        
        //osmanın kırk saniyede uyuduğu fonksiyon
        state.optimize(Rotation2d.fromDegrees(GetRotationPosition()));

        double pidOutput = drivePID.calculate(GetDriveVelocity(), state.speedMetersPerSecond);

        driveMotor.setVoltage(pidOutput + feedforwardVolts);


        rotationPID.setSetpoint(state.angle.getDegrees(), ControlType.kPosition);
    }

    public void stop() {
        driveMotor.stopMotor();
        rotationMotor.stopMotor();
    }

}
