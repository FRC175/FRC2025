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

  public static final double TODDLER_MODE = 0.5;

    public static final class ControllerConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

      

        public static final double test = 0;
    }

    public static final class ManipConstants {
        // PLACEHOLDER VALUES. do not test until we know FOR SURE that these are correct
        public static final double DEFAULT_POSITION = 0.317919;
        public static final double INTAKE_POSITION = 0.319952;
        public static final double FLIPPED_POSITION = .8333;

        public static final double LOWER_LIMIT = 0.290622;
        public static final double UPPER_LIMIT = 0.838988;
    }

    public static final class ElevatorConstants {
        // PLACEHOLDER VALUES. do not test until we know FOR SURE that these are correct
        public static final double MAX_HEIGHT = 1100;
        public static final double MIN_HEIGHT = 25.4;
    }

    

    public static final class DriveConstants {
        public static final int frontRightDrive = 3;
        public static final int frontRightRot = 2;
        public static final int frontLeftDrive = 7;
        public static final int frontLeftRot = 6;
        public static final int backRightDrive = 5;
        public static final int backRightRot = 4;
        public static final int backLeftDrive = 9;
        public static final int backLeftRot = 8;

        public static final double frontRightTurnAngle = 45;
        public static final double frontLeftTurnAngle = 135;
        public static final double backRightTurnAngle = 315;
        public static final double backLeftTurnAngle = 225;

        public static final int frontRightEncoder = 11;
        public static final int frontLeftEncoder = 13;
        public static final int backRightEncoder = 12;
        public static final int backLeftEncoder = 14;

        public static final double frontRightBaseAngle = 0;
        public static final double frontLeftBaseAngle = 0; //0.470215
        public static final double backRightBaseAngle = 0;
        public static final double backLeftBaseAngle = 0;


        public static final int PIDGEON = 10;

        public static final double ENCODERTOANGLE = (double) 360/4096;

    }
    public enum elevatorSetpoint {
        GROUND(25.4),
        L1(321),
        L2(321),
        L3(478),
        L4(722);
        

        double value;
        elevatorSetpoint(double value) {
                this.value = value;
        }

        public double getSetpoint() {
            return value;
        }


    }

    public enum manipulatorSetpoint {
        CORAL(0.317919),
        ALGAE( .8333),
        INTAKING(0.319952);
        

        double value;
        manipulatorSetpoint(double value) {
                this.value = value;
        }

        public double getSetpoint() {
            return value;
        }


    }

    public static final class LEDConstants {
        public static final int BLINKIN_PORT = 12;
    }
    

    

}
    


