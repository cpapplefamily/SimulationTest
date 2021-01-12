// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;

/** Add your docs here. */
public class Drivetrain {
    private static final double kWheelRadius = Units.inchesToMeters(2);
    private static final int kEcoderResolution = 4096;

    private final PWMVictorSPX m_leftLeader = new PWMVictorSPX(1);
    private final PWMVictorSPX m_leftFollower = new PWMVictorSPX(2);
    private final PWMVictorSPX m_rightLeader = new PWMVictorSPX(3);
    private final PWMVictorSPX m_rightFollower = new PWMVictorSPX(4);


    private SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(m_leftLeader, m_leftFollower);
    private SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(m_rightLeader, m_rightFollower);

    private final Encoder m_leftEncoder = new Encoder(0,1);
    private final Encoder m_rightEncoder = new Encoder(2,3);

    private final AnalogGyro m_gyro = new AnalogGyro(0);

    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    //Simmulation Classes to help us simulate
    private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    private final EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);

    private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

    //Create a Simulation Model of the Drivtrain
    DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
        DCMotor.getNEO(2),          //driveMotor, 
        7.25,                       //gearing, 
        7.5,                        //jKgMetersSquared, 
        60,                         //massKg, 
        kWheelRadius,               //wheelRadiusMeters, 
        0.7112,                     //trackWidthMeters, 
        null                        //measurementStdDevs
        );

    private final Field2d m_field = new Field2d();

    public Drivetrain(){
        //Set the distance travled per Revolution of the encoder
        m_leftEncoder.setDistancePerPulse( 2 * Math.PI * kWheelRadius / kEcoderResolution);
        m_rightEncoder.setDistancePerPulse( 2 * Math.PI * kWheelRadius / kEcoderResolution);

        m_rightGroup.setInverted(true);
        
        m_leftEncoder.reset();
        m_rightEncoder.reset();

        SmartDashboard.putData("Field", m_field);
    }

    public void drivePercent(double left, double right){
        SmartDashboard.putNumber("left Speed", m_leftLeader.getSpeed());
        SmartDashboard.putNumber("right Speed", m_rightLeader.getSpeed());
        m_leftGroup.set(left);
        m_rightGroup.set(right);
    }

    public void updateOdometry(){
        SmartDashboard.putNumber("left Encoder", m_leftEncoder.getDistance());
        SmartDashboard.putNumber("right Encoder", m_rightEncoder.getDistance());
        SmartDashboard.putNumber("m_gyro", m_gyro.getRotation2d().getDegrees());
        m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    }

    public void resetOdometry(Pose2d pose){
        m_leftEncoder.reset();
        m_rightEncoder.reset();
        m_driveSim.setPose(pose);
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    public double getDistanceMeters(){
        return(m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2;
    }

    public Pose2d getPose(){
        return m_odometry.getPoseMeters();
    }

    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
        SmartDashboard.putNumber(" m_leftLeader", m_leftLeader.get());
        SmartDashboard.putNumber(" RobotController.getInputVoltage()", RobotController.getInputVoltage());
        m_driveSim.setInputs(
            m_leftLeader.get() * RobotController.getInputVoltage(), 
            -m_rightLeader.get() * RobotController.getInputVoltage());
        m_driveSim.update(0.02);

        m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
        m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
        m_rightEncoderSim.setRate(m_driveSim.getRightPositionMeters());
        m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());

    }

    public void periodic(){
        updateOdometry();
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }





}