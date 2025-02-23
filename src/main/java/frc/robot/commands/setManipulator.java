package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;
import frc.robot.Constants.ManipConstants;

public class setManipulator extends Command{
    private final Manipulator manipulator;
    private final double goalAngle, currentAngle, deadband;
    
    

    
    public setManipulator(double deadband) {
      this.manipulator = Manipulator.getInstance();
      this.deadband = deadband;
      
      
      if (manipulator.isFlipped()) {
        goalAngle = ManipConstants.DEFAULT_POSITION;
      } else {
        goalAngle = ManipConstants.FLIPPED_POSITION;
      }

      currentAngle = goalAngle;
      
    }

    @Override
    public void execute() {
      double currentAngle = manipulator.getEncoder();
      manipulator.invertFlip();
      if (currentAngle - deadband < goalAngle || currentAngle + deadband > goalAngle) {
        manipulator.setFlipOpenLoop(0.2);
      }

      




        

    }
    
    @Override
    public void end(boolean interrupted) {
       manipulator.setFlipped(!manipulator.isFlipped());
        
    }
    @Override
    public boolean isFinished() {
     return (currentAngle - deadband < goalAngle || currentAngle + deadband > goalAngle);
    }

}
