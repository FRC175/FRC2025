package frc.robot.commands.manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ManipConstants;
import frc.robot.Constants.manipulatorSetpoint;
import frc.robot.subsystems.Manipulator;
import frc.robot.Constants.ManipConstants;

public class setManipWorking extends Command {
    
   private final Manipulator manipulator; 
   private final double upSpeed, downSpeed;
   private double upperLimit, lowerLimit;
   private double goalPoint;
   private final double deadband;
   private boolean surpassedLimit;
    

    public setManipWorking(Manipulator manipulator, double deadband, double upSpeed, double downSpeed) {
        this.manipulator = manipulator;
        this.upSpeed = upSpeed;
        this.downSpeed = downSpeed;
        this.deadband = deadband;
        upperLimit = ManipConstants.UPPER_LIMIT;
        lowerLimit = ManipConstants.LOWER_LIMIT;
        goalPoint = manipulator.getGoalSetpoint();
        addRequirements(manipulator);
    }

    @Override
    public void initialize() {
        surpassedLimit = false;
    }


    @Override
    public void execute() {
        goalPoint = manipulator.getGoalSetpoint();
        double currentAngle = manipulator.getEncoder();
        if (currentAngle < goalPoint - deadband) {
          manipulator.setFlipOpenLoop(-upSpeed);
        } else if (currentAngle > goalPoint + deadband) {
          manipulator.setFlipOpenLoop(downSpeed);
        } else {
          manipulator.setFlipOpenLoop(calculateProportionalOutput(currentAngle, goalPoint));
        }
     }

    private double calculateProportionalOutput(double angle, double goalPoint) {
        double slope = (downSpeed + upSpeed) / (2 * deadband);
        double output = (angle - goalPoint) * slope;
        return output;
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