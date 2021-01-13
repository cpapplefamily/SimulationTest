// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.text.html.HTMLDocument.HTMLReader.PreAction;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  
  private final Drivetrain m_drivetrain = new Drivetrain();
  
  private final double kSetpoint = 3;

  private final PIDController m_pidController = new PIDController(10, 0, 0.0);

  @Override
  public void robotInit() {
    SmartDashboard.putNumber("kP", 10);
    SmartDashboard.putNumber("kd", 0.0);
  }

   
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Setpoint", kSetpoint);
    SmartDashboard.putNumber("Current Position", m_drivetrain.getDistanceMeters());

    m_drivetrain.periodic();
  }

  
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_pidController.setP(SmartDashboard.getNumber("kp", 10));
    m_pidController.setD(SmartDashboard.getNumber("kd", 0));
 
    double output = m_pidController.calculate(m_drivetrain.getDistanceMeters(), kSetpoint);
    m_drivetrain.drivePercent(output, output);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double speed = -m_controller.getRawAxis(1);// getY(Hand.kLeft);
    double rot = -m_controller.getRawAxis(0); // getX(Hand.kLeft) * 0.4;

    m_drivetrain.drivePercent(speed - rot, speed + rot);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
   Pose2d pose =  new Pose2d(2, 2, new Rotation2d());
    m_drivetrain.resetOdometry(pose);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationPeriodic() {
    m_drivetrain.simulationPeriodic();
  }

}


