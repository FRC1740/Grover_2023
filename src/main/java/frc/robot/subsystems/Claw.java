// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// Comment to force commit
// Using "import static an.enum.or.constants.inner.class.*;" helps reduce verbosity
// this replaces "DoubleSolenoid.Value.kForward" with just kForward
// further reading is available at https://www.geeksforgeeks.org/static-import-java/
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class Claw extends SubsystemBase {
  DoubleSolenoid m_solenoid;  

  /** Creates a new Manipulator. */
  public Claw() {
    m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.kManipulatorExtendPort, Constants.kManipulatorRetractPort);
    // Set the colors appropriate for each game piece
  }

  // These next two methods are temporary just to test peumatics.
  // The actual robot may have TWO separate mechanisms for cone/cube
  // selected by the drive team via OI input TBD
  public void Deploy() {
    m_solenoid.set(kForward);
  }

  public void Retract() {
    m_solenoid.set(kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}