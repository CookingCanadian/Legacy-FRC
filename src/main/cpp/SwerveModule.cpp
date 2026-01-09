package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private final double m_angleOffset;
    private final TalonFX m_driveMotor;
    private final TalonFX m_steerMotor;
    private final CANcoder m_encoder;
    private final PIDController m_steerPID;

    private final DutyCycleOut m_driveRequest = new DutyCycleOut(0);
    private final DutyCycleOut m_steerRequest = new DutyCycleOut(0);

    private static final double kWheelDiameter = 0.1016; // meters
    private static final double kGearRatio = 6.2;

    public SwerveModule(int driveID, int steerID, int encoderID, double offset) {
        m_angleOffset = offset;
        m_driveMotor = new TalonFX(driveID);
        m_steerMotor = new TalonFX(steerID);
        m_encoder = new CANcoder(encoderID);
        m_steerPID = new PIDController(0.1, 0.0, 0.0002);

        // Configure drive motor
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_driveMotor.getConfigurator().apply(driveConfig);

        // Configure steer motor
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Test this
        m_steerMotor.getConfigurator().apply(steerConfig);

        // Enable continuous input for steer PID (wraps around at -π to π)
        m_steerPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setDesiredState(SwerveModuleState state) {
        // Get current encoder position
        var encoderSignal = m_encoder.getAbsolutePosition();
        encoderSignal.refresh();
        double encoderTurns = encoderSignal.getValue();

        // Convert encoder turns to radians and apply offset
        double currentAngle = encoderTurns * 2.0 * Math.PI;
        double adjustedAngle = currentAngle - m_angleOffset;

        // Optimize the state to minimize wheel rotation
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            state, 
            new Rotation2d(adjustedAngle)
        );

        // Calculate drive output as percentage of max speed
        double driveOutput = optimizedState.speedMetersPerSecond / AutoConstants.kMaxSpeed;

        // Calculate steer output using PID
        double steerError = optimizedState.angle.getRadians() - adjustedAngle;
        double steerOutput = m_steerPID.calculate(steerError, 0.0);

        // Set motor outputs
        m_driveMotor.setControl(m_driveRequest.withOutput(driveOutput));
        m_steerMotor.setControl(m_steerRequest.withOutput(steerOutput));
    }

    public SwerveModulePosition getPosition() {
        // Get drive motor position (in rotations)
        double drivePosition = m_driveMotor.getPosition().getValue();
        
        // Convert rotations to meters
        double distance = drivePosition * (kWheelDiameter * Math.PI) / kGearRatio;

        // Get current angle
        double encoderTurns = m_encoder.getAbsolutePosition().getValue();
        double currentAngle = encoderTurns * 2.0 * Math.PI;
        double adjustedAngle = currentAngle - m_angleOffset;

        return new SwerveModulePosition(distance, new Rotation2d(adjustedAngle));
    }

    public SwerveModuleState getState() {
        // Get drive motor velocity (in rotations per second)
        double driveVelocity = m_driveMotor.getVelocity().getValue();
        
        // Convert to meters per second
        double speed = driveVelocity * (kWheelDiameter * Math.PI) / kGearRatio;

        // Get current angle
        double encoderTurns = m_encoder.getAbsolutePosition().getValue();
        double currentAngle = encoderTurns * 2.0 * Math.PI;
        double adjustedAngle = currentAngle - m_angleOffset;

        return new SwerveModuleState(speed, new Rotation2d(adjustedAngle));
    }
}