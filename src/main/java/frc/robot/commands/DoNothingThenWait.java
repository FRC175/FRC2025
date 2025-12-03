package frc.robot.commands;

import frc.robot.commands.DoNothing;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.BreadMotor;

public class DoNothingThenWait extends SequentialCommandGroup {
   
    private BreadMotor breadMotor;
    public DoNothingThenWait(double time, BreadMotor breadMotor) {
        this.breadMotor = BreadMotor.getInstance();
        addCommands(
            new DoNothing(breadMotor),
            new WaitCommand(time)
        );
    }

   

  

}