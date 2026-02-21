#pragma once

#include <frc/XboxController.h>
#include <frc2/command/button/Trigger.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/SPI.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <studica/AHRS.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <rev/SparkMax.h>
#include <units/length.h>
#include <numbers>
#include <memory>

namespace OperatorConstants {
    constexpr int kSwerveControllerPort = 0;
    constexpr int kElevatorControllerPort = 1;
    constexpr double kDeadband = 0.08;
    constexpr int kStrafeAxis = frc::XboxController::Axis::kLeftX;
    constexpr int kForwardAxis = frc::XboxController::Axis::kLeftY;
    constexpr int kRotationAxis = frc::XboxController::Axis::kRightX;
    constexpr int kFieldRelativeButton = frc::XboxController::Button::kRightBumper;
}

class SwerveModule {
public:
    SwerveModule(int driveID, int steerID, int encoderID, double offset);
    void SetDesiredState(const frc::SwerveModuleState& state);
    frc::SwerveModuleState GetState();
    frc::SwerveModulePosition GetPosition();

private:
    double m_angleOffset;
    ctre::phoenix6::hardware::TalonFX m_driveMotor;
    ctre::phoenix6::hardware::TalonFX m_steerMotor;
    ctre::phoenix6::hardware::CANcoder m_encoder;
    frc::PIDController m_steerPID{0.005, 0.03, 0.0002}; 
};

namespace AutoConstants {
    constexpr auto kMaxSpeed = 4_mps;
    constexpr auto kMaxAcceleration = 6_mps_sq;
    constexpr auto kMaxAngularSpeed = std::numbers::pi * 1_rad_per_s;
    constexpr auto kMaxAngularAcceleration = std::numbers::pi * 2_rad_per_s_sq;

    constexpr double kPXController = 0.5;
    constexpr double kPYController = 0.5;
    constexpr double kPThetaController = 0.5;

    const frc::TrapezoidProfile<units::radians>::Constraints
        kThetaControllerConstraints{kMaxAngularSpeed, kMaxAngularAcceleration};
}

class RobotContainer {
public:
    RobotContainer();
    void Drive(double xSpeed, double ySpeed, double rot, bool fieldRelative);
    void UpdateOdometry();
    void SetMechanismPosition(double joystickY);
    frc::Pose2d GetPose() const; // Added public method to access odometry pose

    std::unique_ptr<studica::AHRS> m_navx;
    frc::XboxController m_swerveController{OperatorConstants::kSwerveControllerPort};
    frc::XboxController m_elevatorController{OperatorConstants::kElevatorControllerPort};

private:
    static constexpr double kWheelBase = 0.495;
    static constexpr double kTrackWidth = 0.495;

    SwerveModule m_frontLeft{5, 1, 9, 0}; // do the angle tuning in phoenix tuner (the right most parameter) 
    SwerveModule m_frontRight{6, 2, 10, 0}; 
    SwerveModule m_backLeft{8, 4, 12, 0};   
    SwerveModule m_backRight{7, 3, 11, 0};                      

    rev::spark::SparkMax m_elevatorPivot{13, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax m_elevatorPivot2{14, rev::spark::SparkMax::MotorType::kBrushless};

    frc::SwerveDriveKinematics<4> m_kinematics{
        frc::Translation2d{units::meter_t{kWheelBase/2}, units::meter_t{kTrackWidth/2}},
        frc::Translation2d{units::meter_t{kWheelBase/2}, units::meter_t{-kTrackWidth/2}},
        frc::Translation2d{units::meter_t{-kWheelBase/2}, units::meter_t{kTrackWidth/2}},
        frc::Translation2d{units::meter_t{-kWheelBase/2}, units::meter_t{-kTrackWidth/2}}
    };

    frc::SwerveDriveOdometry<4> m_odometry;
};