package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;


public class SwerveSubsytem extends SubsystemBase {

    SwerveDrive swerveDrive;

    public SwerveSubsytem (File directory) {
           // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    double maximumSpeed = Units.feetToMeters(4.5);
    File swerveJsonDirectory = directory;
    try {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.DriveConstants.MAX_SPEED);
    } catch (IOException e) {
        throw new RuntimeException(e);
    }
    }

    @Override
    public void simulationPeriodic()
    {
    }

    public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

     /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

   /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

   /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

   



  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
//   public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
//                               DoubleSupplier headingY)
//   {
//     return run(() -> {

//       Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
//                                                                                  translationY.getAsDouble()), 0.8);

//       // Make the robot move
//       swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
//                                                                       headingX.getAsDouble(),
//                                                                       headingY.getAsDouble(),
//                                                                       swerveDrive.getOdometryHeading().getRadians(),
//                                                                       swerveDrive.getMaximumChassisVelocity()));
//     });
//   }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
//   public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
//   {
//     return run(() -> {
//       // Make the robot move
//       swerveDrive.drive(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
//                                           translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
//                         angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
//                         true,
//                         false);
//     });
//   }

  
  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }


}