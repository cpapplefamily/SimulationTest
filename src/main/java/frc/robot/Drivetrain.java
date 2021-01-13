// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
import frc.robot.sim.PhysicsSim;


/** Add your docs here. */
public class Drivetrain {
    private static final double kWheelRadius = Units.inchesToMeters(2);
    private static final int kEcoderResolution = 4096;

    private final WPI_TalonSRX m_leftLeader = new WPI_TalonSRX(1);
    private final WPI_TalonSRX m_leftFollower1 = new WPI_TalonSRX(2);
    private final WPI_TalonSRX m_leftFollower2 = new WPI_TalonSRX(3);
    private final WPI_TalonSRX m_rightLeader = new WPI_TalonSRX(4);
    private final WPI_TalonSRX m_rightFollower1 = new WPI_TalonSRX(5);
    private final WPI_TalonSRX m_rightFollower2 = new WPI_TalonSRX(6);


    private SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(m_leftLeader, m_leftFollower1,m_leftFollower2);
    private SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(m_rightLeader, m_rightFollower1,m_rightFollower2);

    private final Encoder m_leftEncoder = new Encoder(0,1);
    private final Encoder m_rightEncoder = new Encoder(2,3);

    private final AnalogGyro m_gyro = new AnalogGyro(0);

    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    //Simmulation Classes to help us simulate
    private final EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    private final EncoderSim m_rightEncoderSim = new EncoderSim(new Encoder(2,3));

    private final AnalogGyroSim m_gyroSim = new AnalogGyroSim(m_gyro);

    //Create a Simulation Model of the Drivtrain
    DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
        DCMotor.getFalcon500(3),          //driveMotor, 
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
        m_leftGroup.set(left);
        m_rightGroup.set(right);
    }

    public void updateOdometry(){
        m_odometry.update(m_gyro.getRotation2d(), 
        m_leftEncoder.getDistance(), 
        m_rightEncoder.getDistance()
        );
    }

    public void resetOdometry(Pose2d pose){
        m_leftEncoder.reset();
        m_rightEncoder.reset();
        m_driveSim.setPose(pose);
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    public double getDistanceMeters(){
        return(m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2;
        //return(m_leftLeader.getSelectedSensorPosition(0) + m_rightLeader.getSelectedSensorPosition(0)) / 2;
    }

    public Pose2d getPose(){
        return m_odometry.getPoseMeters();
    }

    
    public void simulationInit() {
    //    PhysicsSim.getInstance().addTalonSRX(m_leftLeader, 0.75, 4000, true);
    //    PhysicsSim.getInstance().addTalonSRX(m_rightLeader, 0.75, 4000, true);
        //PhysicsSim.getInstance().addTalonSRX(_rghtFollower, 0.75, 4000);
        //PhysicsSim.getInstance().addTalonSRX(_leftFollower, 0.75, 4000);
    }

    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
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
