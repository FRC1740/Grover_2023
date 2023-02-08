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
    // One rotation of the output = 1.5 inches of extension
    public static final double kArmExtensionRotationToLinearDistance = 5;
    // Output rotation in 15-1 system: 15 rotations of the motor = 1 rotation of output
    // One rotation = 1.5 inches
    public static final double ARM_EXTENSION_POSITION_CONVERSION_FACTOR = kArmExtensionRotationToLinearDistance/kArmExtensionGearRatio; //Encoder output in inches of extension


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
    public static final double kSubStationAngle = 70; // Started 75

    // Arm Extension Position Constants
    public static final double kStowedPosition = 0;    // FIXME: Pseudo-wild guess at node distance
    public static final double kHighNodePosition = 1;  // Start with 4 These values came from CAD and will likely
    public static final double kMidNodePosition = 2;  // 28 change once the arm is installed on the robot
    public static final double kLowNodePosition = 3;  // Was 4. Use 12 for testing
    public static final double kSubStationPosition = 12; // FIXME: Substation extension? 

    // ARM Rotation PID constants
    public static final double kRotP = .02;
    public static final double kRotI = 0;
    public static final double kRotD = 0;

    // ARM Rotation PID constants
    public static final double kExtP = .01;
    public static final double kExtI = 0;
    public static final double kExtD = 0;


}
