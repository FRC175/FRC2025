package frc.robot.commands.manipulator;

import java.util.concurrent.TransferQueue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;
import frc.robot.Constants.ManipConstants;

public class setManipulator extends Command{
    private final Manipulator manipulator;
    private double goalAngle, currentAngle;
    private final double deadband, lowerLimit, upperLimit;
    private boolean surpassedLimit, manual,manualTweak, cc;
  
    
    

    
    public setManipulator(Manipulator manipulator, double deadband, boolean flip) {
      this.manipulator = manipulator;
      this.deadband = deadband;
      lowerLimit = ManipConstants.LOWER_LIMIT;
      upperLimit = ManipConstants.UPPER_LIMIT;
      surpassedLimit = false;
      this.manualTweak = false;
      this.cc = manipulator.cc;

      addRequirements(manipulator);
      
      
     if (!manualTweak) { 
      currentAngle= manipulator.getEncoder();
       if (flip) {
        if (manipulator.isFlipped()) {
          goalAngle = ManipConstants.DEFAULT_POSITION;
        } else {
          goalAngle = ManipConstants.FLIPPED_POSITION;
        }

       } else {
        goalAngle = currentAngle;
       }
      } else {
        currentAngle= manipulator.getEncoder();
       if (!cc) {
        goalAngle = (currentAngle += .005);
       } else {
        goalAngle = (currentAngle -= .005);
       }
      }
      
    }

    @Override
    public void execute() {
      System.out.println("HI!!!! I CAN SEE YOUR INPUTS  YOUR CODE SUCKS!!");
      double currentAngle = manipulator.getEncoder();
      System.out.println(surpassedLimit);
      if (currentAngle <= upperLimit || currentAngle >= lowerLimit) {
        manipulator.setFlipOpenLoop(0);
        surpassedLimit = true;
      }
      if  (!(surpassedLimit || (currentAngle - deadband < goalAngle || currentAngle + deadband > goalAngle))){

      if (!manual) {
        manipulator.invertFlip();
      }
      if (currentAngle - deadband < goalAngle || currentAngle + deadband > goalAngle) {
        manipulator.setFlipOpenLoop(-0.3);
      } 
    } else {
      manipulator.setFlipOpenLoop(0);
      manualTweak = false;
      if (!surpassedLimit) {
        manipulator.setFlipped(!manipulator.isFlipped());
       }
    }

      




        

    }
    
    @Override
    public void end(boolean interrupted) {
     if (!manual) {
        manipulator.setFlipOpenLoop(0);
     }
     if (!surpassedLimit) {
      manipulator.setFlipped(!manipulator.isFlipped());
     }
     manualTweak = false;
     }
   
      
      
        
    @Override
    public boolean isFinished() {
     
     return false;
    }

}
