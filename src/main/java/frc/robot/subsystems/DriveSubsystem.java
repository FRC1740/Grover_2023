// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;
import frc.robot.OI;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.*;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveSubsystem extends SubsystemBase {

  // ADIS16448 plugged into the MXP port
  ADIS16448_IMU m_gyro = new ADIS16448_IMU();  

  /** Creates a new DriveSubsystem. */
  private final WPI_TalonSRX m_leftMotorLeader;
  private final WPI_TalonSRX m_leftMotorFollower;
  private final WPI_TalonSRX m_rightMotorLeader;
  private final WPI_TalonSRX m_rightMotorFollower;

  // The robot's drive
  private final DifferentialDrive m_drive;

  GenericEntry m_nte_DriveSpeedFilter;
  GenericEntry m_nte_DriveRotationFilter;

  // Drive input filters
  LinearFilter speedFilter;
  LinearFilter rotationFilter;
  
  public DriveSubsystem() {

    /* Master Talons for arcade drive */
    m_leftMotorLeader = new WPI_TalonSRX(1);
    m_rightMotorLeader = new WPI_TalonSRX(4);

    m_leftMotorLeader.setInverted(false);
    m_rightMotorLeader.setInverted(true);

    /* Follower Talons + Victors for six motor drives */
    m_leftMotorFollower = new WPI_TalonSRX(3);
    m_rightMotorFollower = new WPI_TalonSRX(2);

    m_leftMotorFollower.follow(m_leftMotorLeader);
    m_rightMotorFollower.follow(m_rightMotorLeader);    

    m_leftMotorFollower.setInverted(InvertType.FollowMaster);
    m_rightMotorFollower.setInverted(InvertType.FollowMaster);

    m_drive = new DifferentialDrive(m_leftMotorLeader, m_rightMotorLeader);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable m_nt = inst.getTable("drivetrain");
    

    // Create and get reference to SB tab
    ShuffleboardTab m_sbt_DriveTrain = edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab("DriveTrain");
  
    // GenericEntry m_nte_Testing;
  
    // Create widgets for digital filter lengths
    m_nte_DriveSpeedFilter = m_sbt_DriveTrain.addPersistent("Drive Speed Filter", 11)
          .withSize(2, 1).withPosition(0, 0).getEntry();

    m_nte_DriveRotationFilter = m_sbt_DriveTrain.addPersistent("Drive Rotation Filter", 5)
          .withSize(2, 1).withPosition(0, 1).getEntry();

    speedFilter = LinearFilter.movingAverage((int)m_nte_DriveSpeedFilter.getInteger(11));
    rotationFilter = LinearFilter.movingAverage((int)m_nte_DriveSpeedFilter.getInteger(5));
      
    //  m_nte_Testing     = m_sbt_DriveTrain.addPersistent("Testing", 0.0)       .withSize(1, 1).withPosition(3, 3).getEntry();

  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  
   public void arcadeDrive(double fwd, double rot, boolean squaredInput) {
    double f_fwd = speedFilter.calculate(fwd);
    double f_rot = rotationFilter.calculate(rot);
    // Swap the next two lines for filtered/raw input
    m_drive.arcadeDrive(OI.deadZone(fwd), OI.deadZone(rot), squaredInput);
    //m_drive.arcadeDrive(OI.deadZone(f_fwd), OI.deadZone(f_rot), squaredInput);
    
  }
  
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public double getAngle() {
    //System.out.println("gyro angle" + m_gyro.getAngle());
    return m_gyro.getAngle();
  }

  public Rotation2d getRotation2d(){
    return Rotation2d.fromDegrees(m_gyro.getAngle());
  }  

  public void resetGyro() {
    m_gyro.reset();
  }

  public double getRoll(){
    return m_gyro.getGyroAngleY();
  }
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return m_gyro.getAngle();
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
