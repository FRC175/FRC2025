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
    
        public static final double driveDeadbandX = 0.3;
        public static final double driveDeadbandY = 0.3;
        public static final double driveDeadbandTwist = 0.3;



        public static final double ENCODERTOANGLE = (double) 360/4096;

        public static final double MAXIMUMSPEED = Units.feetToMeters(11);


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
    


