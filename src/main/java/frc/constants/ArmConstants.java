package frc.constants;

public class ArmConstants {
    
    public static final int kRotationLeaderMotorPort = 6; //Right side of the arm
    public static final int kRotationFollowerMotorPort = 7; //Left side of the arm
    public static final int kExtensionMotorPort = 8;

    // Arm Rotation Constants
    public static final double kArmRotationGearRatio = 144; // Gear boxes 4x3x3, Sprockets 64/16 = 4 total 4x3x3x4 = 144
    public static final double ARM_ROTATION_POSITION_CONVERSION_FACTOR = 360/kArmRotationGearRatio; //Encoder output in degrees the arm rotates
    // Arm Extension Constants
    public static final double kArmExtensionGearRatio = 15; //Gear box is 5x3, Sprockets 1/1
    // One rotation of the output = 5 inches of extension
    public static final double kArmExtensionOutputToInches = 5;
    // Encoder output to inches of extension
    public static final double ARM_EXTENSION_POSITION_CONVERSION_FACTOR = kArmExtensionOutputToInches/kArmExtensionGearRatio;

    /*
        * All Angles based on Horizontal = 0
        * Starting Configuration: 112 deg (0); Fully Retracted
        * Mid Node Scoring: 41 deg (71); 4"  Extension
        * High Node Scoring: 37 deg (75); 28" Extension
        * Low Node Scoring: -22 deg (134); 4" Extension
        * Human player maybe same as mid-node (close)
        */
    // Arm Rotation Angle constants
    public static final double kStowedAngle = 0;
    public static final double kHighNodeAngle = 66; // FIXME: Started at 71; Remounted w/ hardstop
    public static final double kMidNodeAngle = 70;  // Started: 75
    public static final double kLowNodeAngle = 129; // Started 134
    public static final double kSubStationAngle = 70; // Same as Mid-node? [Started 75]

    // Arm Extension Position Constants
    public static final double kStowedPosition = 0;    // FIXME: Pseudo-wild guess at node distance
    public static final double kHighNodePosition = 4;  // These values came from CAD and will likely
    public static final double kMidNodePosition = 20;  // 28 change once the arm is installed on the robot
    public static final double kLowNodePosition = 8;    // started at 4
    public static final double kSubStationPosition = 28; // Same as Mid-node? 

    // ARM Rotation PID constants
    public static final double kRotP = .01;
    public static final double kRotI = 0;
    public static final double kRotD = 0;

    // ARM Extension PID constants
    public static final double kExtP = .02;
    public static final double kExtI = 0;
    public static final double kExtD = 0;


}
