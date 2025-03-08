package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveToDist;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.utils.Utils;

public class leave extends SequentialCommandGroup {

    public leave(Drive drive) {
        addCommands(
        new InstantCommand(() -> drive.resetGyro(180)),
           new SwerveToDist(drive, .3, 180, 180, 12 )
        );

        
    }
    public leave(Drive drive, double speed, double dist, double headingAngle, double transAngle) {
            this( drive);
        }
    }