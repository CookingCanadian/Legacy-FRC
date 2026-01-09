package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    private static final String kAutoNameDefault = "Default";
    private static final String kAutoNameCustom = "Custom";
    
    private RobotContainer robotContainer;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    private String m_autoSelected;
    private double autoStartTime;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        m_chooser.addOption(kAutoNameDefault, kAutoNameDefault);
        m_chooser.addOption(kAutoNameCustom, kAutoNameCustom);
        m_chooser.setDefaultOption(kAutoNameDefault, kAutoNameDefault);
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    @Override
    public void robotPeriodic() {
        robotContainer.updateOdometry();
    }

    @Override
    public void teleopInit() {
        robotContainer.getNavX().zeroYaw();
    }

    @Override
    public void teleopPeriodic() {
        double xSpeed = -robotContainer.getSwerveController().getRawAxis(OperatorConstants.kForwardAxis);
        double ySpeed = robotContainer.getSwerveController().getRawAxis(OperatorConstants.kStrafeAxis);
        double rot = robotContainer.getSwerveController().getRawAxis(OperatorConstants.kRotationAxis);
        boolean fieldRelative = robotContainer.getSwerveController().getRawButton(OperatorConstants.kFieldRelativeButton);

        double triggerValue = robotContainer.getSwerveController().getLeftTriggerAxis();
        double speedScale = 1.0;
        if (triggerValue > 0.2) {
            speedScale = 1.0 - ((triggerValue - 0.2) / 0.8) * 0.8;
        }

        xSpeed *= speedScale;
        ySpeed *= speedScale;
        rot *= speedScale;

        robotContainer.drive(xSpeed, ySpeed, rot, fieldRelative);

        double mechanismY = -robotContainer.getElevatorController().getRawAxis(XboxController.Axis.kLeftY.value);
        robotContainer.setMechanismPosition(mechanismY);
    }

    @Override
    public void autonomousInit() {
        autoStartTime = Timer.getFPGATimestamp();
        m_autoSelected = m_chooser.getSelected();
    }

    @Override
    public void autonomousPeriodic() {
        double now = Timer.getFPGATimestamp();
        double elapsed = now - autoStartTime;

        if (elapsed < 2.0) {
            robotContainer.drive(0, 0.1, 0, true);
            var pose = robotContainer.getPose();
            if (pose.getY() > 0.5) {
                robotContainer.drive(0, 0, 0, true);
            }
        } else {
            robotContainer.drive(0, 0, 0, true);
        }
    }

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}
}