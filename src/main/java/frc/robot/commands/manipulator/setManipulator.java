package frc.robot.commands.manipulator;

import java.util.concurrent.TransferQueue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;
import frc.robot.Constants.ManipConstants;

public class setManipulator extends Command{
    private final Manipulator manipulator;
    private final double goalAngle, currentAngle, deadband, lowerLimit, upperLimit;
    private boolean surpassedLimit;
    
    

    
    public setManipulator(double deadband) {
      this.manipulator = Manipulator.getInstance();
      this.deadband = deadband;
      lowerLimit = ManipConstants.LOWER_LIMIT;
      upperLimit = ManipConstants.UPPER_LIMIT;
      surpassedLimit = false;
      
      
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
      if (currentAngle >= upperLimit || currentAngle <= lowerLimit) {
        manipulator.setFlipOpenLoop(0);
        surpassedLimit = true;
      }
      manipulator.invertFlip();
      if (currentAngle - deadband < goalAngle || currentAngle + deadband > goalAngle) {
        manipulator.setFlipOpenLoop(0.2);
      }

      




        

    }
    
    @Override
    public void end(boolean interrupted) {
      manipulator.setFlipOpenLoop(0);
     if (!surpassedLimit) {
      manipulator.setFlipped(!manipulator.isFlipped());
     }
      
      
        
    }
    @Override
    public boolean isFinished() {
     
     return (surpassedLimit || (currentAngle - deadband < goalAngle || currentAngle + deadband > goalAngle));
    }

}
