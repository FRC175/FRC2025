package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipConstants;
import frc.robot.Constants.manipulatorSetpoint;
import frc.robot.subsystems.Manipulator;
import frc.robot.Constants.ManipConstants;

public class setManipWorking extends Command {
    
   private final Manipulator manipulator; 
   private final double demand;
   private double goalAngle;
   private double upperLimit, lowerLimit;
   private manipulatorSetpoint goalSet, CurrentSet;
   private final double deadband;
   private boolean surpassedLimit;
    

    public setManipWorking(Manipulator manipulator, double deadband, double demand) {
        this.manipulator = manipulator;
        this.demand = demand;
        this.deadband = deadband;
        goalAngle = manipulator.getGoalSetpoint().getSetpoint();
        goalSet = manipulator.getGoalSetpoint();
        upperLimit = ManipConstants.UPPER_LIMIT;
        lowerLimit = ManipConstants.LOWER_LIMIT;
        CurrentSet = manipulator.getCurrentSetpoint();

        addRequirements(manipulator);
    }

    @Override
    public void initialize() {

        manipulator.setGoalSetpoint(goalSet);
    }

    @Override
    public void execute() {
      double currentAngle = manipulator.getEncoder();
      System.out.println(surpassedLimit);
      if (currentAngle <= upperLimit || currentAngle >= lowerLimit) {
        manipulator.setFlipOpenLoop(0);
        surpassedLimit = true;
      }
      if  (!(surpassedLimit || (currentAngle - deadband < goalAngle || currentAngle + deadband > goalAngle))){
        if (currentAngle - deadband < goalAngle) {
            manipulator.setFlipOpenLoop(-0.3);
        } else if (currentAngle + deadband > goalAngle) {
            manipulator.setFlipOpenLoop(0.3);
        }
      } else {
         manipulator.setFlipOpenLoop(0);
          tryFlipIntake();

     }

    }

    private void tryFlipIntake() {
        if (!surpassedLimit && (CurrentSet != goalSet)) {
            
            if(CurrentSet == manipulatorSetpoint.CORAL && goalSet == manipulatorSetpoint.ALGAE) {
                manipulator.setFlipped(true);
            }
            if (CurrentSet == manipulatorSetpoint.ALGAE && (goalSet == manipulatorSetpoint.CORAL || goalSet == manipulatorSetpoint.INTAKING)) {
                manipulator.setFlipped(false);
            }

            manipulator.setCurrentSetpoint(goalSet);
        }
    }

  
    @Override
    public void end(boolean interrupted) {
        manipulator.setFlipOpenLoop(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}