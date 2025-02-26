package frc.robot.commands.manipulator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulator;
import frc.robot.Constants.ManipConstants;

public class DischargeAlgae extends Command{
    private final Manipulator manipulator;
    private boolean discharging;
    private double demand;
   
    
   
    

    
    public DischargeAlgae(double demand) {
      this.manipulator = Manipulator.getInstance();
      discharging = false;
      this.demand = demand;
      

      
     
    }

    @Override
    public void execute() {
        manipulator.setIntakeOpenLoop(demand);
          discharging = true;
        
      
    }
    
    @Override
    public void end(boolean interrupted) {
        manipulator.setIntakeOpenLoop(0);
        
    }
    @Override
    public boolean isFinished() {
     return (false);
    }

}
