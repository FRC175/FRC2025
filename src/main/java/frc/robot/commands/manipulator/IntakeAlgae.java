package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
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
      this.demand = Demand;
      

      
     
    }

    @Override
    public void execute() {
        manipulator.setIntakeOpenLoop(demand);
        intaking = true;

        
    }
    
    @Override
    public void end(boolean interrupted) {
        manipulator.setIntakeOpenLoop(0);
        
    }
    @Override
    public boolean isFinished() {
     return (intaking && manipulator.getIntakeSpeed() <= .1);
    }

}
