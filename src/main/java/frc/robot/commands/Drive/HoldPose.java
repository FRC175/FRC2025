package frc.robot.commands.Drive;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.Drive;
public class HoldPose extends Command{
    private final Drive drive;
   
    
   
    
   
    

    
    public HoldPose(double demand) {
      this.drive = Drive.getInstance();
      
      

      
     
    }

    @Override
    public void initialize() {
      
      
    }

    @Override
    public void execute() {
    drive.setPrevYaw(drive.getYaw());
     drive.resetGyro(0);

      
    }
    
    @Override
    public void end(boolean interrupted) {
       
    }
    @Override
    public boolean isFinished() {
      return false;
    }

}
