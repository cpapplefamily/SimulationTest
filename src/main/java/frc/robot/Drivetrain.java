// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
    private final WPI_TalonSRX m_leftFollower = new WPI_TalonSRX(2);
    private final WPI_TalonSRX m_rightLeader = new WPI_TalonSRX(3);
    private final WPI_TalonSRX m_rightFollower = new WPI_TalonSRX(4);

    //private final WPI_TalonFX m_leftLeader = new WPI_TalonFX(1);
    //private final WPI_TalonFX m_leftFollower = new WPI_TalonFX(2);
    //private final WPI_TalonFX m_rightLeader = new WPI_TalonFX(3);
    //private final WPI_TalonFX m_rightFollower = new WPI_TalonFX(4);


    private SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(m_leftLeader, m_leftFollower);
    private SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(m_rightLeader, m_rightFollower);



    private final AnalogGyro m_gyro = new AnalogGyro(0);

    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    //Simmulation Classes to help us simulate
    private final double setDistancePerPulse = ( 2 * Math.PI * kWheelRadius / kEcoderResolution);


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
        //m_leftEncoder.setDistancePerPulse( 2 * Math.PI * kWheelRadius / kEcoderResolution);
        //m_rightEncoder.setDistancePerPulse( 2 * Math.PI * kWheelRadius / kEcoderResolution);

        m_rightGroup.setInverted(true);
        
        m_leftLeader.setSelectedSensorPosition(0);
        m_rightLeader.setSelectedSensorPosition(0);

        SmartDashboard.putData("Field", m_field);
    }

    public void drivePercent(double left, double right){
        m_leftGroup.set(left);
        m_rightGroup.set(right);
    }

    public void updateOdometry(){
        SmartDashboard.putNumber("left Encoder", m_leftLeader.getSelectedSensorPosition());
        SmartDashboard.putNumber("right Encoder", m_rightLeader.getSelectedSensorPosition());
        SmartDashboard.putNumber("m_gyro", m_gyro.getRotation2d().getDegrees());
        m_odometry.update(m_gyro.getRotation2d(), m_leftLeader.getSelectedSensorPosition()*setDistancePerPulse, m_rightLeader.getSelectedSensorPosition()*setDistancePerPulse);
    }

    public void resetOdometry(Pose2d pose){
        m_leftLeader.setSelectedSensorPosition(0);
        m_rightLeader.setSelectedSensorPosition(0);
        m_driveSim.setPose(pose);
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    public double getDistanceMeters(){
        return(m_leftLeader.getSelectedSensorPosition() + m_rightLeader.getSelectedSensorPosition()) / 2;
    }

    public Pose2d getPose(){
        return m_odometry.getPoseMeters();
    }

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonSRX(m_leftLeader, 0.75, 5100, false);
        PhysicsSim.getInstance().addTalonSRX(m_rightLeader, 0.75, 5100, false);
    }
    
    public void simulationPeriodic() {
        PhysicsSim.getInstance().run();

        // This method will be called once per scheduler run when in simulation
        m_driveSim.setInputs(
            m_leftLeader.get() * RobotController.getInputVoltage(), 
            -m_rightLeader.get() * RobotController.getInputVoltage());
        m_driveSim.update(0.02);

        m_leftLeader.getSimCollection().setQuadratureRawPosition((int) (m_driveSim.getLeftPositionMeters()/setDistancePerPulse));
        m_leftLeader.getSimCollection().setQuadratureVelocity((int) (m_driveSim.getLeftVelocityMetersPerSecond()/setDistancePerPulse));
        m_rightLeader.getSimCollection().setQuadratureRawPosition((int) (m_driveSim.getRightPositionMeters()/setDistancePerPulse));
        m_rightLeader.getSimCollection().setQuadratureVelocity((int) (m_driveSim.getRightPositionMeters()/setDistancePerPulse));
        m_gyroSim.setAngle(-m_driveSim.getHeading().getDegrees());

    }

    public void periodic(){
        updateOdometry();
        m_field.setRobotPose(m_odometry.getPoseMeters());
    }





}
