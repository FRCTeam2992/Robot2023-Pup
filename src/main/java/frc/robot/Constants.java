// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.leds.Color;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.manipulator.Waypoint;
import frc.lib.manipulator.Waypoint.OuttakeType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean dataLogging = false;
    public static final boolean debugDashboard = true;

    public static final ControlMode controller1Mode = ControlMode.Guitar;

    public enum ControlMode {
        Guitar,
        XBox;
    }

    public static class LEDColors {
        public static final Color purple = new Color(210, 75, 230);
        public static final Color yellow = new Color(255, 160, 0);
        public static final Color blue = new Color(0, 0, 255);
        public static final Color white = new Color(210, 200, 180);
        public static final Color off = new Color(0, 0, 0);

        public static final Color yellow1 = new Color(255, 253, 225);
        public static final Color yellow2 = new Color(255, 247, 109);
        public static final Color yellow3 = new Color(255, 219, 0);
        public static final Color yellow4 = new Color(255, 186, 0);
        public static final Color yellow5 = new Color(255, 174, 0);

        public static final Color purple1 = new Color(232, 144, 255);
        public static final Color purple2 = new Color(149, 8, 187);
        public static final Color purple3 = new Color(116, 0, 147);
        public static final Color purple4 = new Color(95, 0, 121);
        public static final Color purple5 = new Color(82, 0, 104);
    }

    public static class DrivetrainConstants {
        // Drive Variables
        public static final boolean isFieldCentric = true;
        public static final boolean isVelocityControlled = true;
        public static final boolean isGyroCorrected = true;
        public static final double joystickDeadband = 0.15;
        public static double joystickXYSmoothFactor = 0.5;
        public static double joystickRotationSmoothFactor = 0.5;
        public static double joystickRotationInverseDeadband = 0.14;

        // Length and Width of the Robot in Meters (Inches: 28 x 31.5)
        public static final double swerveWidth = 0.578;
        public static final double swerveLength = 0.667;

        // Max Swerve Speed (Velocity Control)
        public static final double swerveMaxSpeed = 4.5; // (Meters per Second)(2 Slow, 4.5 normal)

        // Swerve Wheels and Gear Ratio
        public static final double driveGearRatio = 6.75;// 6.75:1
        public static final double driveWheelDiameter = 0.098552;

        // Analog Encoder Offsets (Degrees) - Opposite of Raw Reading - Bevel Gear to
        // Right
        public static final double frontLeftOffset = -90.4; //-174.3
        public static final double frontRightOffset = 101.5; //95.0 //90.8
        public static final double rearLeftOffset = -145.1; //180.6 //170.6
        public static final double rearRightOffset = 147.0; //28.3 //31.0

        public static class PIDConstants {
            // Swerve Drive PID (Velocity Control)
            public static final double driveP = 0.05;// .05
            public static final double driveI = 0.0;// .0
            public static final double driveD = 0.01;
            public static final double driveF = 0.047;

            // Swerve Turn PIDs
            public static final double turnP = 0.013; // .013
            public static final double turnI = 0.0;// .0
            public static final double turnD = 0.00005;
        }

        public static class AutoScorePIDConstants {
            public static final double scoreP = 3.25;
            public static final double scoreI = 0.001;
            public static final double scoreD = 0.1;

            public static final double scoreCruise = 4.0; // m / sec
            public static final double scoreAccel = 4.0; // m / sec^2
        }

        // Gyro P
        public static final double driveGyroP = 0.005;

        // Gyro balancing constants
        public static final double gyroRollOffset = -1.9; // degrees -- its robot pitch but navx roll
        public static final double pitchTolerance = 2.0; // degrees -- level if Abs() less than this
        public static final double pitchDeltaTolerance = 0.08; // degrees/20ms robot cycle
        public static final double balanceMoveSpeed = 0.75; // m/sec -- max speed to crawl for final balance
        public static final double balanceWaitTimer = 1.0; // How long to wait before declaring balanced
        public static final double balanceP = 0.03;
        public static final double balanceI = 0.0;
        public static final double balanceD = 0.006;

        // Drive Rotation P
        public static final double driveRotationP = .01;
        public static final double autoAngleThreshold = 0.3;

        // Swerve Module Translations x=.591/2 y=.654/2
        public static final Translation2d frontLeftLocation = new Translation2d(0.289, 0.3335);
        public static final Translation2d frontRightLocation = new Translation2d(0.289, -0.3335);
        public static final Translation2d rearLeftLocation = new Translation2d(-0.289, 0.3335);
        public static final Translation2d rearRightLocation = new Translation2d(-0.289, -0.3335);

        // Swerve X Axis Correction PID (Path Following)
        public static final double xCorrectionP = 10.0;
        public static final double xCorrectionI = 0.0;
        public static final double xCorrectionD = 0.0;

        // Swerve Y Axis Correction PID (Path Following)
        public static final double yCorrectionP = 10.0;
        public static final double yCorrectionI = 0.0;
        public static final double yCorrectionD = 0.0;

        // Swerve Theta Axis Correction PID (Path Following)
        public static final double thetaCorrectionP = 150.0;
        public static final double thetaCorrectionI = 0.0;
        public static final double thetaCorrectionD = 0.0;

        // Max Path Following Drive Speeds
        public static final double maxPathFollowingVelocity = 3.0; // (Meters per Second)
        public static final double maxPathFollowingAcceleration = 2; // (Meters per Second Squared)

        // Max Path Following Turn Speeds
        public static final double maxThetaVelocity = 6.28; // (Radians per Second)
        public static final double maxThetaAcceleration = 6.28; // (Radians per Second Squared)

        // Max speeds where its safe to X wheels
        public static final double maxSpeedToX = 0.25; // m/sec
        public static final double maxTurnToX = 20.0; // degrees/sec

        public static class CanIDs {
            public static int frontLeftDrive = 2;
            public static int frontLeftTurn = 3;
            public static int frontRightDrive = 4;
            public static int frontRightTurn = 5;
            public static int rearLeftDrive = 6;
            public static int rearLeftTurn = 7;
            public static int rearRightDrive = 8;
            public static int rearRightTurn = 9;

            public static int frontLeftEncoder = 3;
            public static int frontRightEncoder = 5;
            public static int rearLeftEncoder = 7;
            public static int rearRightEncoder = 9;
        }

        // Field Coordinates
        public static class FieldSize {
            public static double FIELD_WIDTH_METERS = 8.02;
            public static double FIELD_LENGTH_METERS = 16.04;
        }
    }

    public static class ScoringGridConstants {
        public static final double fieldWidth = 8.0137;

        public static final double autoAlignmentAreaMinXMeters = 1.8;
        public static final double autoAlignmentAreaMaxXMeters = 2.7;
        public static final double autoAlignmentMaxYErrorMeters = 1.0;
        public static final double conePoleOffsetYMeters = 0.559;

        public static class Red {
            public static final double grid1CenterYMeters = 6.9417;
            public static final double grid2CenterYMeters = 5.2657;
            public static final double grid3CenterYMeters = 3.5897;
        }

        public static class Blue {
            public static final double grid6CenterYMeters = 4.424;
            public static final double grid7CenterYMeters = 2.748;
            public static final double grid8CenterYMeters = 1.072;
        }

        public static final double autoStartXCoordMeters = 1.89;
    }

    public static class TowerConstants {
        public static Waypoint scoreFloor = new Waypoint(80.00, OuttakeType.Hybrid);
        public static Waypoint scoreCubeMid = new Waypoint(45.1, OuttakeType.Mid_Cube);
        public static Waypoint normal = new Waypoint(20, OuttakeType.Unknown);

        public static Waypoint cubeGroundIntake = new Waypoint(129.00, OuttakeType.Unknown);
        public static Waypoint cubeWall3GroundIntake = new Waypoint(125.00, OuttakeType.Unknown);

        public static Waypoint midThrowCube = new Waypoint(35.0, OuttakeType.Mid_Cube);
        public static Waypoint rearLowThrowCube = new Waypoint(-79.0, OuttakeType.Rear_Low_Cube);
        public static Waypoint rearMidThrowCube = new Waypoint(-79.0, OuttakeType.Mid_Cube);
        public static Waypoint rearSafePoint = new Waypoint(-72, OuttakeType.Assumed_Cube);

        public static Waypoint singleLoadStation = new Waypoint(-69.00, OuttakeType.Unknown);
    }

    public static class ArmConstants {
        public static class DeviceIDs {
            public static int armMotor = 27;
            public static int armEncoder = 27;
        }

        public static class PIDConstants {
            public static double P = 0.025;
            public static double I = 0.06;
            public static double D = 0.00;
            public static double FF = 0;
        }

        public static class Limits {
            public static double hardStopTop = 115.2;
            public static double hardStopBottom = -85.5;
            public static double softStopTop = 105.0;
            public static double softStopBottom = -75.0;
        }

        public static double gearRatio = 64.0 * (32.0 / 12.0);
        public static double motorEncoderClicksPerDegree = (2048.0 * gearRatio) / 360.0;
        public static double armAngleToleranceDeg = 0.5; // Moves within 1 degree are "close enough"
        public static double CANCoderOffset = 145.6;

        public static class ArmSlopConstants {
            public static double topZoneLowEdge = 12.0; // Above this may be n the slop zone
            public static double topZoneHiEdge = 23.0; // Past the slop zone -- use adjustment
            public static double topZoneAdjustment = 3.5; // Slop adjustment to apply
        }

        public static double holdPositionMaxTime = 120; // Seconds -- How long to prevent backdrive
    }

    public static class ClawConstants {
        public static class DeviceIDs {
            public static int clawMotor = 28;
        }

        public static class PIDConstants {
            public static double P = 0.02;
            public static double I = 0;
            public static double D = 0;
            public static double FF = 0;
        }

        public static double coneCurrent = 0.0;
        public static double cubeCurrent = 0.0;

        public static class Intake {
            public static class Speed {
                public static double cube = 0.8;
                public static double cone = 0.8;
            }

            public static class DelayCyclesAfterBeamBreak {
                public static int cube = 3;
                public static int cone = 24;
            }
        }

        public static double holdPositionMaxTime = 150; // Seconds -- How long to prevent backdrive
        public static double holdPositionPower = 0.075;

    }

    public static class RobotConstants {
        public static class DeviceIDs {
            public static int networkToggleSwitch = 3;
        }
    }
}
