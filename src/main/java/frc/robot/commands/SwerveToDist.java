package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Drive.Drive;

public class SwerveToDist extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drive drive;

    private final double speed;
    private final double targetAngle;
    private final double headingAngle;
    private double targetDistance;
  
  

    public SwerveToDist(Drive drive, double targetSpeed, double targetAngle, double headingAngle, double targetDistance) {
    
        this.drive = drive;
        this.speed = targetSpeed;
        this.targetAngle = targetAngle;
        this.headingAngle = headingAngle;
        this.targetDistance = targetDistance;

        addRequirements(drive);
    }

    public SwerveToDist(Drive drive, double targetSpeed) {
        this.drive = drive;
        this.speed = targetSpeed;
        this.headingAngle = drive.getYaw();
        this.targetAngle = drive.getYaw();
    }

    @Override
    public void initialize() {
        System.out.println("Initializing SwerveToDist for targetDistance: " + targetDistance);
        drive.resetDistance();
        System.out.println("Drive encoder distance after reset: " + drive.getDriveDistance());

    }

    @Override
    public void execute() {
        double joyX = speed * Math.cos(Math.toRadians((targetAngle+90) % 360));
        double joyY = -speed * Math.sin(Math.toRadians((targetAngle+90) % 360));
        drive.lockSwerve(joyX, joyY, headingAngle, drive.getYaw());
        SmartDashboard.putNumber("Distance", drive.getDriveDistance());
        SmartDashboard.putNumber("Goal angle", headingAngle);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setOpenLoop(0, 0);
        drive.resetDistance();
    }

    @Override
    public boolean isFinished() {
        boolean isDone =  Math.abs(drive.getDriveDistance()) >= targetDistance;
        System.out.println("Is finished: " + isDone);
        return Math.abs(drive.getDriveDistance()) >= targetDistance;
    } 
}