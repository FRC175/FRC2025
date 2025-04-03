package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.utils.Controller;
import frc.robot.utils.Utils;

public class RotateInPlace extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drive drive;
    private final double goalDirection;
    private double currentAngle;
    private final Pigeon2 gyro;

    public RotateInPlace(Drive drive, double goalDirection, Pigeon2 gyro) {
        
        this.drive = drive;
        this.goalDirection = goalDirection;
        this.gyro = gyro;
        this.currentAngle = gyro.getYaw().getValueAsDouble();

        addRequirements(drive);
    }

    @Override
    public void initialize() {

    }


    @Override
    public void execute() {
        currentAngle = gyro.getYaw().getValueAsDouble();
        double twistAmt = 0;
        if (goalDirection - currentAngle > 1) twistAmt = .45; else if (goalDirection - currentAngle < 1) twistAmt = -.45;
        double twist = twistAmt * -1;
        drive.swerve(
            0, 
            0, 
            Math.pow((twist * 0.60), 1), 
            drive.getYaw());
        // drive.postEncoders();
        // drive.postYaw();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return (currentAngle <= goalDirection + .5 && currentAngle >= goalDirection - .5 );
    }
}