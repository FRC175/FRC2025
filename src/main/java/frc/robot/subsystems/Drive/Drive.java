// package frc.robot.subsystems.Drive;


// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.DriveConstants;

// import com.ctre.phoenix6.hardware.Pigeon2;



// public final class Drive implements Subsystem {

//     Pigeon2  gyro; // Psuedo-class representing a gyroscope.
//     SwerveModule[]   swerveModules;

//     SwerveModule frontRight, frontLeft, backRight, backLeft;
//     double lastValidAngle;


//     Translation2d frontRightLocation, frontLeftLocation, backRightLocation, backLeftLocation;
//     SwerveDriveKinematics kinematics;
//     SwerveDriveOdometry odometry;
//     Pose2d pose;
 
//     private static Drive instance;

//     private Drive() {
       
//          swerveModules = new SwerveModule[] {
            
//             frontRight = new SwerveModule(DriveConstants.frontRightDrive, DriveConstants.frontRightRot, DriveConstants.frontRightEncoder),
//             frontLeft = new SwerveModule(DriveConstants.frontLeftDrive, DriveConstants.frontLeftRot, DriveConstants.frontLeftEncoder),
//             backRight = new SwerveModule(DriveConstants.backRightDrive, DriveConstants.backRightRot, DriveConstants.backRightEncoder),
//             backLeft = new SwerveModule(DriveConstants.backLeftDrive, DriveConstants.backLeftRot, DriveConstants.backLeftEncoder)
        
//         };
          
//         kinematics = new SwerveDriveKinematics(
//             new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(12.5)), // Front Left
//             new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(-12.5)), // Front Right
//             new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(12.5)), // Back Left
//             new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(-12.5))  // Back Right
//         );
          
//         gyro = new Pigeon2(DriveConstants.PIDGEON);

//         odometry = new SwerveDriveOdometry(
//             kinematics,
//             gyro.getRotation2d(), // returns current gyro reading as a Rotation2d
//             new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
//             // Front-Left, Front-Right, Back-Left, Back-Right
//             new Pose2d(0,0,new Rotation2d()) // x=0, y=0, heading=0
//         );
    
//     }    
//     public void swerve() {

//         // Create test ChassisSpeeds going X = 14in, Y=4in, and spins at 30deg per second.
//         ChassisSpeeds testSpeeds = new ChassisSpeeds(Units.inchesToMeters(14), Units.inchesToMeters(4), Units.degreesToRadians(30));
        
//         // Get the SwerveModuleStates for each module given the desired speeds.
//         SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(testSpeeds);
//         // swerve modules don't have the nessecary commands yet, 
//         swerveModules[0].setState(swerveModuleStates[0]);
//         swerveModules[1].setState(swerveModuleStates[1]);
//         swerveModules[2].setState(swerveModuleStates[2]);
//         swerveModules[3].setState(swerveModuleStates[3]);
//     }

//     public SwerveModulePosition[] getCurrentSwerveModulePositions()
//     {
//         return new SwerveModulePosition[]{
//             new SwerveModulePosition(swerveModules[0].getDistance(), swerveModules[0].getAngle()), // Front-Left
//             new SwerveModulePosition(swerveModules[1].getDistance(), swerveModules[1].getAngle()), // Front-Right
//             new SwerveModulePosition(swerveModules[2].getDistance(), swerveModules[2].getAngle()), // Back-Left
//             new SwerveModulePosition(swerveModules[3].getDistance(), swerveModules[3].getAngle())  // Back-Right
//         };
//     }


           
//     @Override
//     public void periodic()
//     {
//         // Update the odometry every run.
//         odometry.update(gyro.getRotation2d(),  getCurrentSwerveModulePositions());
//     }
       
  


//     public static Drive getInstance() {
//         if (instance == null) {
//             instance = new Drive();
//         }

//         return instance;
//     }

// }
