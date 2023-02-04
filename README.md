# Grover_2023
FRC Robot 2023 Java Code: Command-Based framework
** Updated to 2023.2.1 Release version

This repository runs parallel to our CommandBased_2023 robot code with the following changes:
* Drivetrain uses Talon motor controllers to run CIM motors
* IMU is AnalogDevices ADIS16448 instead of NavX

### Subsystems

#### DriveSubsystem
* Talon controlled 4-Motor (2 Leaders/2 Followers) Drive System Depends upon CTRE vendor library
* 

#### Arm (PID Subsystem)
* 2x SparkMax controlled brushless Neo motors, leader/follower for rotation
* 1x SparkMax controlled Neo motor for extension

#### Claw
* Pneumatic Actuator for open/close
* 1x SparkMax controlled Neo 550 motor for intake/eject gamepiece
* PWM controlled LED light strip

#### ExampleSubsystem
* Spike relay test light (on/off)

#### GroundIntake
* Pneumatic controlled extend/retract
* SparkMax controlled intake/eject motor

#### SignalLED
* PWM controlled LED light strip
### Commands
* AutoBalancePID: Balance based on "roll" axis (NavX Orientation is sideways)
* DriveToDistance: Drive an arbitrary distance
* DriveToDistancePID: Unused
* ExampleCommand: Used for Testing only. Currently a relay controlled light
* TurnToAngle(angle): Unused
* TurnToAngleProfiled(angle): Turn to an arbitrary angle (PID) 

### Command Groups
* AutoDriveSequential: DriveToDistance (onto ramp); AutoBalancePID

## Gradle Note:

In order to capture some compile-time data, we have modified the build.gradle file. Wpilib season updates may overwrite build.gradle. If this happens, you'll need to add the following to the end:

```
task writeBuildProperties() {
	outputs.upToDateWhen { false }  // never mark as up-to-date to ensure that this gets run every time


	doLast {
        def propFile = new File("${project.rootDir}/src/main/resources/build.properties")

        def prop = new Properties()

        prop.setProperty('build.time', (new Date()).format("yyyy_MM_dd HH:mm:ss"))
        prop.setProperty('build.computer', InetAddress.getLocalHost().getHostName())

        propFile.createNewFile();
        prop.store(propFile.newWriter(), null);

        println("task ran")
    }
}

processResources {
	outputs.upToDateWhen { false }  // never mark as up-to-date to ensure that this gets run every time
	dependsOn("writeBuildProperties")
}

```