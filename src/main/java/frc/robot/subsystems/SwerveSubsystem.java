// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Meter;

// import java.io.File;
// import java.lang.reflect.Field;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import swervelib.parser.SwerveControllerConfiguration;
// import swervelib.parser.SwerveDriveConfiguration;
// import swervelib.parser.SwerveParser;
// import swervelib.SwerveDrive;
// import swervelib.SwerveModule;
// import swervelib.math.SwerveMath;
// import swervelib.telemetry.SwerveDriveTelemetry;
// import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.units.measure.Angle;
// import frc.robot.Constants.DriveConstants;
// import com.ctre.phoenix6.hardware.Pigeon2;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathfindingCommand;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.path.PathConstraints;

// import frc.robot.subsystems.PhotonVision;


// public class SwerveSubsystem extends SubsystemBase {

//   private final double maximumSpeed = DriveConstants.MAXIMUMSPEED;
//   // SparkMax frontRight, frontLeft;
//   // RelativeEncoder FLencoder, FRencoder;

//   private final File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
//   private final SwerveDrive  swerveDrive;
//   private final Pigeon2 angie;
//   private PhotonVision photonVision;
   
     
//   /**
//    * Command to drive the robot using translative values and heading as a setpoint.
//    *
//    * @param translationX Translation in the X direction.
//    * @param translationY Translation in the Y direction.
//    * @param headingX     Heading X to calculate angle of the joystick.
//    * @param headingY     Heading Y to calculate angle of the joystick.
//    * @return Drive command.
//    */


//   public SwerveSubsystem(File directory) {

//     // SparkMax frontRight = new SparkMax(2, MotorType.kBrushless);
//     // SparkMax frontLeft = new SparkMax(8, MotorType.kBrushless);
    
//     // RelativeEncoder FLencoder = frontLeft.getEncoder();
//     // RelativeEncoder FRencoder = frontRight.getEncoder();

//     SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
//     angie = new Pigeon2(24);

//     try
//     {
//       swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
//                                                                   new Pose2d(new Translation2d(Meter.of(1),
//                                                                                                 Meter.of(4)),
//                                                                               Rotation2d.fromDegrees(0)));
//       // Alternative method if you don't want to supply the conversion factor via JSON files.
//       // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
//     } catch (Exception e)
//     {
//       throw new RuntimeException(e);
//     }
//     swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
//     swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
//     swerveDrive.setAngularVelocityCompensation(true,
//                                                 true,
//                                                 0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
//     swerveDrive.setModuleEncoderAutoSynchronize(false,
//                                               1);
  

//   }

//   public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
//   {
//     angie = new Pigeon2(24);
//     swerveDrive = new SwerveDrive(driveCfg,
//                                   controllerCfg,
//                                   maximumSpeed,
//                                   new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
//                                              Rotation2d.fromDegrees(0)));
//   }

//   public void resetGyro(int val) {
//     angie.setYaw(val);
//   }

// // field-oriented
//   public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
//                               DoubleSupplier headingY)
//   {
//     return run(() -> {

//       Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
//                                                                                  translationY.getAsDouble()), 0.8);

//       // Make the robot move
//       driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
//                                                                       headingX.getAsDouble(),
//                                                                       headingY.getAsDouble(),
//                                                                       swerveDrive.getOdometryHeading().getRadians(),
//                                                                       swerveDrive.getMaximumChassisVelocity()));
//     });
//   }

//   /**
//    * Command to drive the robot using translative values and heading as angular velocity.
//    *
//    * @param translationX     Translation in the X direction.
//    * @param translationY     Translation in the Y direction.
//    * @param angularRotationX Rotation of the robot to set
//    * @return Drive command.
//    */
//   public Command driveRelativeCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
//   {

//     // System.out.println(FLencoder.getPosition() + " " + FRencoder.getPosition());
    
//     return run(() -> {
//       SwerveModule[] states = swerveDrive.getModules();
//       // for (int i = 0; i < states.length; ++i) {
//       //   System.out.print(i + ": " + states[i].getRawAbsolutePosition() + " ~~~ ");
//       // }
     
//       swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
//                                           translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
//                         angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
//                         true,
//                         false);
//     });
    
//   }

//   public Command driveToPose(Pose2d pose)
//   {
// // Create the constraints to use while pathfinding
//     PathConstraints constraints = new PathConstraints(
//         swerveDrive.getMaximumChassisVelocity(), 4.0,
//         swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

// // Since AutoBuilder is configured, we can use it to build pathfinding commands
//     return AutoBuilder.pathfindToPose(
//         pose,
//         constraints,
//         edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
//                                      );
//   }


//   public void driveFieldOriented(ChassisSpeeds velocity)
//   {
//     swerveDrive.driveFieldOriented(velocity);
//   }

//   @Override
//   public void periodic() {
//     //SmartDashboard.putNumber("gyro", gyri);
//   }

//   public void setupPhotonVision () {
//     photonVision = new PhotonVision(swerveDrive::getPose, swerveDrive.field);
//   }

//   public PhotonVision setupPhotonVisionObject () {
//     return (new PhotonVision(swerveDrive::getPose, swerveDrive.field));
//   }
  
//   public Field2d getField() {
//     return swerveDrive.field;
//   }
//   public Pose2d getPose()
//   {
//     return swerveDrive.getPose();
//   }

//   public void zeroGyro()
//   {
//     swerveDrive.zeroGyro();
//   }

//   public void resetOdometry(Pose2d initialHolonomicPose)
//   {
//     swerveDrive.resetOdometry(initialHolonomicPose);
//   }
//   public ChassisSpeeds getRobotVelocity()
//   {
//     return swerveDrive.getRobotVelocity();
//   }
  
//   public void setupPathPlanner()
//   {
//     // Load the RobotConfig from the GUI settings. You should probably
//     // store this in your Constants file
//     RobotConfig config;
    
    
//     try
//     {
//       config = RobotConfig.fromGUISettings();

//       final boolean enableFeedforward = true;
//       // Configure AutoBuilder last
//       AutoBuilder.configure(
//           this::getPose,
//           // Robot pose supplier
//           this::resetOdometry,
//           // Method to reset odometry (will be called if your auto has a starting pose)
//           this::getRobotVelocity,
//           // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
//           (speedsRobotRelative, moduleFeedForwards) -> {
//             if (enableFeedforward)
//             {
//               swerveDrive.drive(
//                   speedsRobotRelative,
//                   swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
//                   moduleFeedForwards.linearForces()
//                                );
//             } else
//             {
//               swerveDrive.setChassisSpeeds(speedsRobotRelative);
//             }
//           },
//           // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
//           new PPHolonomicDriveController(
//               // PPHolonomicController is the built in path following controller for holonomic drive trains
//               new PIDConstants(5.0, 0.0, 0.0),
//               // Translation PID constants
//               new PIDConstants(5.0, 0.0, 0.0)
//               // Rotation PID constants
//           ),
//           config,
//           // The robot configuration
//           () -> {
//             // Boolean supplier that controls when the path will be mirrored for the red alliance
//             // This will flip the path being followed to the red side of the field.
//             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

//             var alliance = DriverStation.getAlliance();
//             if (alliance.isPresent())
//             {
//               return alliance.get() == DriverStation.Alliance.Red;
//             }
//             return false;
//           },
//           this
//           // Reference to this subsystem to set requirements
//                            );

//     } catch (Exception e)
//     {
//       // Handle exception as needed
//       e.printStackTrace();
//     }

//     //Preload PathPlanner Path finding
//     // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
//     PathfindingCommand.warmupCommand().schedule();
//   }

// }
