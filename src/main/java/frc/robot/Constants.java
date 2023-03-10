// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OI {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
    public static final double DEADZONE = .05;
  }
  public static final int kManipulatorExtendPort = 0;
  public static final int kManipulatorRetractPort = 1;

  public class ShuffleboardConstants {

    public static final String RobotTab = "Robot";
    public static final String ClimberTab = "Climber";
    public static final String DriveTrainTab = "DriveTrain";
    public static final String LauncherTab = "Launcher";
    public static final String IntakeTab = "Intake";
    public static final String VisionTab = "Vision";
  }  

}
