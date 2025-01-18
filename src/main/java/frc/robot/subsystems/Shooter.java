package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ShooterConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class Shooter implements Subsystem {
    private final SparkMax top;
    private final SparkMax bottom;
    private final RelativeEncoder topEncoder, bottomEncoder;
    private final SparkMaxConfig config;
    

    private static Shooter instance;
    
private Shooter() {
    top = new SparkMax(ShooterConstants.TOP, SparkLowLevel.MotorType.kBrushless);
    bottom = new SparkMax(ShooterConstants.BOTTOM, SparkLowLevel.MotorType.kBrushless);
    config = new SparkMaxConfig();
    
    configureSparks();

    topEncoder = top.getEncoder();
    bottomEncoder = bottom.getEncoder();
}

public static Shooter getInstance() {
    if (instance == null) {
        instance = new Shooter();
    }

    return instance;
}

private void configureSparks() {


    

    bottom.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    bottom.setInverted(false);

    top.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    top.setInverted(false);
    
}

public void shooterSetOpenLoop(double demandTop, double demandBot) {
    // SmartDashboard.putNumber("Demand", demand);
    // SmartDashboard.putNumber("Average RPM", getAverageShooterRPM());
    // SmartDashboard.putNumber("Bottom RPM", getBottomRPM());
    // SmartDashboard.putNumber("Top RPM", getTopRPM());
    // System.out.println("Average RPM: " + getAverageShooterRPM());
    // System.out.println("Bottom RPM: " + getBottomRPM());
    // System.out.println("Top RPM: " + getTopRPM());
    top.set(demandTop);
    bottom.set(demandBot);
}

@Override
public void periodic() {
    SmartDashboard.putNumber("Top RPM", getTopRPM());
    SmartDashboard.putNumber("Bottom RPM", getBottomRPM());
}

public double getTopRPM() {
    return topEncoder.getVelocity();
}

public double getBottomRPM() {
    return bottomEncoder.getVelocity();
}

public double getAverageShooterRPM() {
    return (getTopRPM() + getBottomRPM()) / 2;
}

public void turnOffShooter() {
    shooterSetOpenLoop(0, 0);

}

}