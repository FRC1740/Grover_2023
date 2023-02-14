// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.GroundIntake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.constants.ArmConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final Claw m_claw = new Claw();
  public final Arm m_arm = new Arm();
  public final Telescope m_telescope = new Telescope();
  // FIXME: Uncomment the following when GroundIntake is ready to test
  // public final GroundIntake m_groundIntake = new GroundIntake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // The driver's controller
  private final XboxController m_driverController = new XboxController(Constants.OI.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Enable the Arm PID Subsystem
    // Disable this when running ground intake commands    
    m_arm.enable();
    m_telescope.enable();

    // Basic commands to rotate arm to specific set points
    // THE FOLLOWING COMMANDS WORK: DO NOT CHANGE; KEEP FOR REFERENCE

    // new JoystickButton(m_driverController, Button.kA.value)
    //   .onTrue(new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kStowedAngle)));

    // new JoystickButton(m_driverController, Button.kB.value)
    //   .onTrue(new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kHighNodeAngle)));

    // new JoystickButton(m_driverController, Button.kX.value)
    //   .onTrue(new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kMidNodeAngle)));

    // new JoystickButton(m_driverController, Button.kY.value)
    //   .onTrue(new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kLowNodeAngle)));

    // Button commands to test Arm Extension
    // Testing new Telescope PID subsystem/setpoint commands
    new JoystickButton(m_driverController, Button.kA.value)
      .onTrue(new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kStowedPosition)));

    new JoystickButton(m_driverController, Button.kB.value)
      .onTrue(new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kHighNodePosition)));

    new JoystickButton(m_driverController, Button.kX.value)
      .onTrue(new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kMidNodePosition)));

    new JoystickButton(m_driverController, Button.kY.value)
      .onTrue(new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kLowNodePosition)));

    // Combination PID commands for Arm rotate & extend/retract
    // new JoystickButton(m_driverController, Button.kA.value)
    //   .onTrue(new SequentialCommandGroup(
    //       new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kStowedPosition)),
    //       new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kStowedAngle))));

    // new JoystickButton(m_driverController, Button.kB.value)
    //   .onTrue(new SequentialCommandGroup(
    //       new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kHighNodeAngle)),
    //       new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kHighNodePosition))));

    // new JoystickButton(m_driverController, Button.kX.value)
    //   .onTrue(new SequentialCommandGroup(
    //       new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kMidNodeAngle)),
    //       new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kMidNodePosition))));

    // new JoystickButton(m_driverController, Button.kY.value)
    //   .onTrue(new SequentialCommandGroup(
    //       new InstantCommand(() -> m_arm.setSetpoint(ArmConstants.kLowNodeAngle)),
    //       new InstantCommand(() -> m_telescope.setSetpoint(ArmConstants.kLowNodePosition))));
  
    // Configure default commands
    // "Mario-Cart" drive: Triggers are gas and brake. Right stick turns left/right
    // Triggers are Axis 2; RightStick X is axis 3
    m_robotDrive.setDefaultCommand(
        new RunCommand(() ->
            m_robotDrive.arcadeDrive(m_driverController.getRightTriggerAxis() - m_driverController.getLeftTriggerAxis(),
                  -m_driverController.getLeftX(), true), m_robotDrive));

    // m_arm.setDefaultCommand(
    //     new RunCommand(() -> m_arm.telescope(m_driverController.getRightY()), m_arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return null;
  }
}
