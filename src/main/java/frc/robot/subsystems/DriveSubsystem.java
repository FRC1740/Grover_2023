// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.*;

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
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  
   public void arcadeDrive(double fwd, double rot, boolean squaredInput) {
    m_drive.arcadeDrive(fwd, rot, squaredInput);
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
