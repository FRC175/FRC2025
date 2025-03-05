// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.ManipConstants;
// import frc.robot.subsystems.Manipulator;

// public class setManipWorking extends Command {
    
//    private final Manipulator arm; 
//    private final double downSpeed;
//    private final double upSpeed;
//    private double manipPos
//    private double armGoalPosition;
//    private final double UP_DEADBAND = 0.1;
//    private final double DOWN_DEADBAND = 0.05;

//     public setManipWorking(Arm arm, double downSpeed, double upSpeed) {
//         this(arm, downSpeed, upSpeed, arm.getArmGoalPosition());
//     }
    
//     public setManipWorking(Arm arm, double downSpeed, double upSpeed, ArmPosition armGoal) {
//         this(arm, downSpeed, upSpeed, armGoal.getPosition());
//     }

//     /**
//      * This command drives the arm indefinitely towards its goal point. The up and down speeds
//      * are set independently to optimize the travel time and minimize error.
//      * @param arm the arm subsystem
//      * @param downSpeed the speed to drive the arm when moving down (outside the deadband)
//      * @param upSpeed the speed to drive the arm when moving up (outside the deadband)
//      * @param goal the target position for the arm, as an encoder count, to be determined
//      *             experimentally
//      */
//     public setManipWorking(Arm arm, double downSpeed, double upSpeed, double goal) {
//         this.arm = arm;
//         this.downSpeed = downSpeed;
//         this.upSpeed = upSpeed;
//         this.armGoalPosition = goal;
//         addRequirements(arm);
//     }

//     @Override
//     public void initialize() {
//         /*
//          * The arm goal position is set within the arm subsystem when the command initializes. This
//          * goal position can later be changed within the arm subsystem allowing this command to drive
//          * to a different setpoint without being interrupted.
//          */
//         arm.setArmGoalPosition(armGoal);
//     }

//     @Override
//     public void execute() {
//         /*
//          * If the arm is far enough away from the setpoint, i.e. outside the upper and lower deadbands,
//          * then it is simply driven at its full up or down speed. Otherwise, a more complex function is
//          * used to calculate the speed such that it approaches the goal quickly but without overshoot or
//          * oscillation.
//          */
//         armGoalPosition = arm.getArmGoalPosition();

//         if (arm.getPosition() > armGoalPosition + UP_DEADBAND) {
//             arm.setArmOpenLoop(-upSpeed);
//         } else if (arm.getPosition() < armGoalPosition - DOWN_DEADBAND) {
//             arm.setArmOpenLoop(+downSpeed);
//         } else {
//             arm.setArmOpenLoop(calculateDeadbandSpeed(armGoalPosition));
//         }
//     }

//     public double calculateDeadbandSpeed(double armGoalPosition) {
//         /*
//          * This function sets the arm motor speed to be proportional to the distance remaining to the
//          * goal set point, and is used once the arm is relatively close to the goal, i.e. within both
//          * deadbands.
//          */
//         if (arm.getPosition() > armGoalPosition) {
//             return upSpeed - (upSpeed / UP_DEADBAND) * (arm.getPosition() - (armGoalPosition - UP_DEADBAND));
//         } 
//         if (arm.getPosition() < armGoalPosition) {
//             return downSpeed - (downSpeed / DOWN_DEADBAND) * (arm.getPosition() - (armGoalPosition - DOWN_DEADBAND));
//         }
//         return 0;
//     }
  
//     @Override
//     public void end(boolean interrupted) {
//         arm.setArmOpenLoop(0);
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }