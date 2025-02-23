package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;
import frc.robot.Constants.ManipConstants;

public class IntakeAlgae extends Command{
    private final Manipulator manipulator;
    private boolean intaking;
    private double demand;
   
    
   
    

    
    public IntakeAlgae(double Demand) {
      this.manipulator = Manipulator.getInstance();
      intaking = false;
      this.demand = -Demand;
      

      
     
    }

    @Override
    public void execute() {
      boolean upstream = manipulator.isUpstream();
      boolean downStream = manipulator.isDownstream();
     
        manipulator.setIntakeOpenLoop(demand);
        
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
