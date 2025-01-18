// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;

// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.ArmPosition;

// public class Arm implements Subsystem {

//     private static Arm instance; 

//     private final CANSparkMax armMaster, armSlave;
    
//     private double armGoalPosition;
//     private final DutyCycleEncoder armEncoder; 

//     private Arm() {
//         armMaster = new CANSparkMax(ArmConstants.ARM_LEFT, CANSparkMax.MotorType.kBrushless);
//         armSlave = new CANSparkMax(ArmConstants.ARM_RIGHT, CANSparkMax.MotorType.kBrushless);
//         armEncoder = new DutyCycleEncoder(1);
//         System.out.println("encoder position: " + armEncoder.getAbsolutePosition());
        
//         armGoalPosition = ArmPosition.INTAKE.getPosition();
        
//         configureSparks();
//     }

//     public static Arm getInstance() {
// 		if (instance == null) {
// 			instance = new Arm();
// 		}

// 		return instance;
// 	}

//     private void configureSparks() {
//         armMaster.restoreFactoryDefaults();
//         // armMaster.setInverted(false);

        
//         armSlave.restoreFactoryDefaults();
//         // armSlave.setInverted(true);
//         // armSlave.follow(armMaster); 
        
//     }

//     @Override
//     public void periodic() {
//         // SmartDashboard.putNumber("Current Master", armMaster.getOutputCurrent());
//         // SmartDashboard.putNumber("Current Slave", armSlave.getOutputCurrent());
//         // SmartDashboard.putNumber("Arm Position", getPosition());
//         // SmartDashboard.putNumber("Goal Arm Position: ", getArmGoalPosition());
//         SmartDashboard.putNumber("encoder position: "  , armEncoder.getAbsolutePosition());
        
        
//     }
    
//     public void setArmOpenLoop(double demand) {
//         armMaster.set(-demand);
//         armSlave.set(demand);
//     }

//     public void setMasterLoop(double demand) {
//         armMaster.set(-demand);
//     }

//     public void setSlaveLoop(double demand) {
//         armSlave.set(demand);
//     }



//     public double getPosition() {
//         return armEncoder.getAbsolutePosition();
        
//     }

//     public double getArmGoalPosition() {
//         return armGoalPosition;
//     }

//     public void setArmGoalPosition(ArmPosition armGoalPosition) {
//         this.armGoalPosition = armGoalPosition.getPosition();
//     }

//     public void setArmGoalPosition(double armGoalPosition) {
//         this.armGoalPosition = armGoalPosition;
//     }

//     public void resetEncoders() {
//         armEncoder.setPositionOffset(0.909);
//         armEncoder.reset();
//     }
// }
