// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static double TODDLER_MODE = 0.5;

    public static final class ControllerConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final double CONTROLLER_DEADBAND = 0.1;

        public static final double test = 0;
    }

    

    public static final class DriveConstants {
        public static final int frontRightDrive = 7;
        public static final int frontRightRot = 8;
        public static final int frontLeftDrive = 9;
        public static final int frontLeftRot = 6;
        public static final int backRightDrive = 3;
        public static final int backRightRot = 2;
        public static final int backLeftDrive = 5;
        public static final int backLeftRot = 4;

        public static final double frontRightTurnAngle = 45;
        public static final double frontLeftTurnAngle = 135;
        public static final double backRightTurnAngle = 315;
        public static final double backLeftTurnAngle = 225;

        public static final int frontRightEncoder = 21;
        public static final int frontLeftEncoder = 22;
        public static final int backRightEncoder = 23;
        public static final int backLeftEncoder = 24;

        public static final double frontRightBaseAngle = 0.318604;
        public static final double frontLeftBaseAngle = -0.0356;
        public static final double backRightBaseAngle = 0.208740;
        public static final double backLeftBaseAngle = 0.599609;

        public static final double driveDeadbandX = 0.1;
        public static final double driveDeadbandY = 0.1;


        public static final int PIDGEON = 20;

        public static final double ENCODERTOANGLE = (double) 360/4096;

        public static final double MAXIMUMSPEED = Units.feetToMeters(4.5);


    }

    public static final class IntakeConstants {
        public static final int INTAKE = 12; 
    }

    public static final class ShooterConstants {
        public static final int TOP = 13;
        public static final int BOTTOM = 14;
    }

    public static final class LiftConstants {
        public static final int LEFT = 16;
        public static final int RIGHT = 15; 
    }

    public static final class ArmConstants {
        public static final int ARM_RIGHT = 10; 
        public static final int ARM_LEFT = 11; 
    }

    public enum ArmPosition {
        REST(0.700),
        SPEAKER(0.840),
        PODIUM(0.7955),
        INTAKE(0.8975),
        AMP(0.630),
        PASSING(0.785),
        START_NOTE(0.8),
        TRAP(0.8650),
        WHATEVER(0.82),
        
        MARRR(0.79);

        private final double position;

        private ArmPosition(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
        


    }

    public static final class LEDConstants {
        public static final int BLINKIN_PORT = 0;
    }
    public static final class LimelightConstants {

        public static final double mountHeight = 14.628;
        public static final double mountAngle = 36.10;
        public static final double[] tagHeights = {48.125, 48.125, 51.875, 51.875, 48.125, 48.125, 48.125, 48.125, 51.875, 51.875, 47.5, 47.5, 47.5, 47.5, 47.5, 47.5,};

    }

}
    


