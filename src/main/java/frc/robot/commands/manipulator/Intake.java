package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ManipConstants;

public class Intake extends Command{
    private final Manipulator manipulator;
    private final Elevator elevator;
    private boolean intaking, upStream, downStream;
    private double highDemand, lowDemand;
    
   
    
   
    

    
    public Intake(double highDemand, double lowDemand) {
      this.manipulator = Manipulator.getInstance();
      this.elevator = Elevator.getInstance();
      intaking = false;
      upStream = false;
      downStream = false;
      this.highDemand = -highDemand;
      this.lowDemand = -lowDemand;
      

      
     
    }

    @Override
    public void execute() {
      if (!manipulator.isFlipped()) {
        boolean upstream = manipulator.isUpstream();
        boolean downStream = manipulator.isDownstream();
        elevator.coralInPeril = true;
      
          manipulator.setIntakeOpenLoop(highDemand);
          if (upstream) {
            intaking = true;
            manipulator.setIntakeOpenLoop(lowDemand);
          }
          if (downStream) {
            intaking = true;
            manipulator.setIntakeOpenLoop(-lowDemand); 
          }
        } else {
          manipulator.setIntakeOpenLoop(-highDemand);
          intaking = true;

        }
    }
    
    @Override
    public void end(boolean interrupted) {
      elevator.coralInPeril = false;
        manipulator.setIntakeOpenLoop(0);
        
    }
    @Override
    public boolean isFinished() {

    if (!manipulator.isFlipped()) {
      return (!upStream && !downStream && intaking);
    } else {
      return (intaking && manipulator.getIntakeSpeed() <= .1);
    }
      
    }

}
