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

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final WPI_TalonSRX m_leftMotorLeader;
  private final WPI_TalonSRX m_leftMotorFollower;
  private final WPI_TalonSRX m_rightMotorLeader;
  private final WPI_TalonSRX m_rightMotorFollower;

  // The robot's drive
  private final DifferentialDrive m_drive;

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
    
    // get a topic from a NetworkTableInstance
    // the topic name in this case is the full name
    //DoubleTopic dblTopic = inst.getDoubleTopic("/drivetrain/gyro");
    
    // get a topic from a NetworkTable
    // the topic name in this case is the name within the table;
    // this line and the one above reference the same topic
    // DoubleTopic dtGyro = m_nt.getDoubleTopic("gyro");
    
    // get a type-specific topic from a generic Topic
    // Topic genericTopic = inst.getTopic("/datatable/X");
    // DoubleTopic dblTopic = new DoubleTopic(genericTopic);

    // Create and get reference to SB tab
    ShuffleboardTab m_sbt_DriveTrain = edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab("DriveTrain");
  
    GenericEntry m_nte_Testing;
  
    // Autonomous Variables
    GenericEntry m_nte_a_DriveDelay;
    GenericEntry m_nte_b_DriveDistance;
    GenericEntry m_nte_c_DriveTurnAngle;
    GenericEntry m_nte_autoDriveMode;


    // Create widgets for digital filter lengths
    GenericEntry m_nte_DriveSpeedFilter = m_sbt_DriveTrain.addPersistent("Drive Speed Filter", 10.0)
          .withSize(2, 1).withPosition(0, 0).getEntry();

    GenericEntry m_nte_DriveRotationFilter = m_sbt_DriveTrain.addPersistent("Drive Rotation Filter", 5.0)
          .withSize(2, 1).withPosition(0, 1).getEntry();

    // Create widget for non-linear input
    GenericEntry m_nte_InputExponent = m_sbt_DriveTrain.addPersistent("Input Exponent", 1.0)        .withSize(1, 1).withPosition(0, 2).getEntry();

    // Create widgets for AutoDrive
    m_nte_a_DriveDelay     = m_sbt_DriveTrain.addPersistent("a Launch Delay", .5)
          .withSize(1, 1).withPosition(3, 0).getEntry();
    m_nte_b_DriveDistance  = m_sbt_DriveTrain.addPersistent("b Drive Distance", 48)
          .withSize(1, 1).withPosition(3, 1).getEntry();
    m_nte_c_DriveTurnAngle = m_sbt_DriveTrain.addPersistent("c Turn Angle", 0.0)
          .withSize(1, 1).withPosition(3, 2).getEntry();
    m_nte_autoDriveMode    = m_sbt_DriveTrain.addPersistent("AutoDrive Mode", 2)
          .withSize(1, 1).withPosition(3, 3).getEntry();

    //  m_nte_Testing     = m_sbt_DriveTrain.addPersistent("Testing", 0.0)       .withSize(1, 1).withPosition(3, 3).getEntry();

    // Encoder outputs
    // Display current encoder values
    GenericEntry m_nte_LeftEncoder = m_sbt_DriveTrain.addPersistent("Left Side Encoder", 0.0)
                .withSize(2,1).withPosition(4,0).getEntry();

    GenericEntry m_nte_RightEncoder = m_sbt_DriveTrain.addPersistent("Right Side Encoder", 0.0)
              .withSize(2,1).withPosition(4,1).getEntry();

    GenericEntry m_nte_IMU_ZAngle = m_sbt_DriveTrain.addPersistent("IMU Z-Axis Angle", 0.0)
              .withSize(2,1).withPosition(4,2).getEntry();

  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  
   public void arcadeDrive(double fwd, double rot, boolean squaredInput) {
    m_drive.arcadeDrive(OI.deadZone(fwd), OI.deadZone(rot), squaredInput);
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
