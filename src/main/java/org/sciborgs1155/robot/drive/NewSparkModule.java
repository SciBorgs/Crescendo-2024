package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.lib.FaultLogger.*;
import static org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.COUPLING_RATIO;
import java.util.Set;

import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Data;
import org.sciborgs1155.lib.SparkUtils.Sensor;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Driving;
import org.sciborgs1155.robot.drive.DriveConstants.ModuleConstants.Turning;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class NewSparkModule implements ModuleIO {
    private final CANSparkFlex driveMotor; // Neo Vortex
    private final CANSparkMax turnMotor; // Neo 550

    private final RelativeEncoder driveEncoder;
    private final SparkAbsoluteEncoder turningEncoder;

    private final SparkPIDController drivePID;
    private final SparkPIDController turnPID;

    private final SimpleMotorFeedforward driveFF;
    private final Rotation2d angularOffset;

    private double lastPosition;
    private double lastVelocity;

    private SwerveModuleState setpoint = new SwerveModuleState();
    
    public final String name;
    
    public NewSparkModule(int drivePort, int turnPort, Rotation2d angularOffset, String name) {
        driveMotor = new CANSparkFlex(drivePort, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        drivePID = driveMotor.getPIDController();
        driveFF = new SimpleMotorFeedforward(Driving.FF.S, Driving.FF.V, Driving.FF.kA_linear); // TODO: Re-tune probably?
        
        // TODO: Re-tune
        drivePID.setP(Driving.PID.P);
        drivePID.setI(Driving.PID.I);
        drivePID.setD(Driving.PID.D);

        check(driveMotor, driveMotor.restoreFactoryDefaults());
        check(driveMotor, driveMotor.setIdleMode(IdleMode.kBrake));
        check(driveMotor, driveMotor.setSmartCurrentLimit((int) Driving.CURRENT_LIMIT.in(Amps)));
        check(driveMotor, driveEncoder.setPositionConversionFactor(Driving.POSITION_FACTOR.in(Meters)));
        check(
            driveMotor,
            driveEncoder.setVelocityConversionFactor(Driving.VELOCITY_FACTOR.in(MetersPerSecond)));
        check(driveMotor, driveEncoder.setAverageDepth(16));
        check(driveMotor, driveEncoder.setMeasurementPeriod(32));
        check(
            driveMotor,
            SparkUtils.configureFrameStrategy(
                driveMotor,
                Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
                Set.of(Sensor.INTEGRATED),
                false));
        check(driveMotor, driveMotor.burnFlash());

        turnMotor = new CANSparkMax(turnPort, MotorType.kBrushless);
        turningEncoder = turnMotor.getAbsoluteEncoder();
        turnPID = turnMotor.getPIDController();

        // TODO: Re-tune
        turnPID.setP(Turning.PID.P);
        turnPID.setI(Turning.PID.I);
        turnPID.setD(Turning.PID.D);

        check(turnMotor, turnMotor.restoreFactoryDefaults());
        check(turnMotor, turnMotor.setIdleMode(IdleMode.kBrake));
        check(turnMotor, turnMotor.setSmartCurrentLimit((int) Turning.CURRENT_LIMIT.in(Amps)));
        turningEncoder.setInverted(Turning.ENCODER_INVERTED);
        check(turnMotor);
        check(
            turnMotor, turningEncoder.setPositionConversionFactor(Turning.POSITION_FACTOR.in(Radians)));
        check(
            turnMotor,
            turningEncoder.setVelocityConversionFactor(Turning.VELOCITY_FACTOR.in(RadiansPerSecond)));
        check(turnMotor, turningEncoder.setAverageDepth(2));
        check(
            turnMotor,
            SparkUtils.configureFrameStrategy(
                turnMotor,
                Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
                Set.of(Sensor.ABSOLUTE),
                false));
        SparkUtils.addChecker(
            () ->
                check(
                    turnMotor,
                    SparkUtils.configureFrameStrategy(
                        turnMotor,
                        Set.of(Data.POSITION, Data.VELOCITY, Data.APPLIED_OUTPUT),
                        Set.of(Sensor.ABSOLUTE),
                        false)));
        check(turnMotor, turnMotor.burnFlash());

        register(driveMotor);
        register(turnMotor);

        resetEncoders();

        this.angularOffset = angularOffset;
        this.name = name;
    }

    @Override
    public void setDriveVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
        check(driveMotor);
        log("current", driveMotor.getOutputCurrent());
    }

    @Override
    public void setTurnVoltage(double voltage) {
        turnMotor.setVoltage(voltage);
        check(turnMotor);
    }

    @Override
    public double drivePosition() {
        lastPosition = SparkUtils.wrapCall(driveMotor, driveEncoder.getPosition()).orElse(lastPosition);
        // account for rotation of turn motor on rotation of drive motor
        return lastPosition - turningEncoder.getPosition() * COUPLING_RATIO;
    }

    @Override
    public double driveVelocity() {
        lastVelocity = SparkUtils.wrapCall(driveMotor, driveEncoder.getVelocity()).orElse(lastVelocity);
        return lastVelocity;
    }

    @Override
    public Rotation2d rotation() {
        return Rotation2d.fromRadians(turningEncoder.getPosition()).minus(angularOffset);
    }

    @Override
    public void resetEncoders() {
        driveEncoder.setPosition(0);
    }

    @Override
    public SwerveModuleState state() {
        return new SwerveModuleState(driveVelocity(), rotation());
    }

    @Override
    public SwerveModulePosition position() {
        return new SwerveModulePosition(drivePosition(), rotation());
    }

    @Override
    public SwerveModuleState desiredState() {
        return setpoint;
    }

    @Override
    public void setDriveSetpoint(double velocity) {
        drivePID.setReference(velocity, ControlType.kVelocity);
    }

    @Override
    public void setTurnSetpoint(double angle) {
       turnPID.setReference(angle, ControlType.kPosition);
    }

    @Override
    public void updateSetpoint(SwerveModuleState setpoint, ControlMode mode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateSetpoint'");
    }

    @Override
    public void close() {
        driveMotor.close();
        turnMotor.close();
    }
}
