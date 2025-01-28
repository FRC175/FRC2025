package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;


public class SwerveSubsystem extends SubsystemBase {

  private final double maximumSpeed = DriveConstants.MAXIMUMSPEED;
  // SparkMax frontRight, frontLeft;
  // RelativeEncoder FLencoder, FRencoder;

  private final File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
  private final SwerveDrive  swerveDrive;
   
     
  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */


  public SwerveSubsystem(File directory) {

    // SparkMax frontRight = new SparkMax(2, MotorType.kBrushless);
    // SparkMax frontLeft = new SparkMax(8, MotorType.kBrushless);
    
    // RelativeEncoder FLencoder = frontLeft.getEncoder();
    // RelativeEncoder FRencoder = frontRight.getEncoder();

  SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

  try
  {
    swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
                                                                new Pose2d(new Translation2d(Meter.of(1),
                                                                                              Meter.of(4)),
                                                                            Rotation2d.fromDegrees(0)));
    // Alternative method if you don't want to supply the conversion factor via JSON files.
    // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
  } catch (Exception e)
  {
    throw new RuntimeException(e);
  }
  swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
  swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
  swerveDrive.setAngularVelocityCompensation(true,
                                              true,
                                              0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
  swerveDrive.setModuleEncoderAutoSynchronize(false,
                                              1);
  

  }

  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg)
  {
    swerveDrive = new SwerveDrive(driveCfg,
                                  controllerCfg,
                                  maximumSpeed,
                                  new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
                                             Rotation2d.fromDegrees(0)));
  }


  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                                                                                 translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {

    // System.out.println(FLencoder.getPosition() + " " + FRencoder.getPosition());
    
    return run(() -> {
      SwerveModule[] states = swerveDrive.getModules();
      for (int i = 0; i < states.length; ++i) {
        System.out.print(i + ": " + states[i].getRawAbsolutePosition() + " ~~~ ");
      }
      System.out.println();

      System.out.println(angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity());
      swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                          translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
    
  }

  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

}
