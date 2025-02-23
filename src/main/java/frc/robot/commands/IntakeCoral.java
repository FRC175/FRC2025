package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;
import frc.robot.Constants.ManipConstants;

public class IntakeCoral extends Command{
    private final Manipulator manipulator;
    private boolean intaking, upStream, downStream;
    private double highDemand, lowDemand;
   
    
   
    

    
    public IntakeCoral(double highDemand, double lowDemand) {
      this.manipulator = Manipulator.getInstance();
      intaking = false;
      upStream = false;
      downStream = false;
      this.highDemand = -highDemand;
      this.lowDemand = -lowDemand;
      

      
     
    }

    @Override
    public void execute() {
      boolean upstream = manipulator.isUpstream();
      boolean downStream = manipulator.isDownstream();
     
        manipulator.setIntakeOpenLoop(highDemand);
        if (upstream) {
          intaking = true;
          manipulator.setIntakeOpenLoop(lowDemand);
        }
        if (downStream) {
          intaking = true;
          manipulator.setIntakeOpenLoop(-lowDemand); 
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        manipulator.setIntakeOpenLoop(0);
        
    }
    @Override
    public boolean isFinished() {
     return (!upStream && !downStream && intaking);
    }

}
