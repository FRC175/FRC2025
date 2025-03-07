package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorSetpoint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Elevator extends SubsystemBase {
    private static Elevator instance;
    private final SparkMax master, slave;
    private final SparkMaxConfig defaultConfig;
    private final ResetMode resetMode;
    private final PersistMode persistMode;
    private final LaserCan distSensor;
    public boolean coralInPeril;
    public boolean coralOverride;
    public boolean manual;
    private final DigitalInput topProxSwitch, botProxSwitch;
    private double goalPoint = 0.0;

    public Elevator() {
        this.master = new SparkMax(15, MotorType.kBrushless);
        this.slave = new SparkMax(16, MotorType.kBrushless);
        this.defaultConfig = new SparkMaxConfig();
        this.resetMode = SparkBase.ResetMode.kResetSafeParameters;
        this.persistMode = PersistMode.kPersistParameters;
        this.distSensor = new LaserCan(21);
        coralOverride = false;
        this.botProxSwitch = new DigitalInput(3);
        this.topProxSwitch = new DigitalInput(4);

        defaultConfig.inverted(false);
        configureSparks();
        configureDistSensor();

        goalPoint = ElevatorSetpoint.L1.getSetpoint();
    }
    
    @Override
    public void periodic() {
       SmartDashboard.putNumber("ele dist", getDistance());
       //System.out.println("Goal: " + getGoalSetpoint());
    }

    public boolean isTopProxMade () {
        return topProxSwitch.get();
    }

    public boolean isBotProxMade () {
        return botProxSwitch.get();
    }

    public static Elevator getInstance() {
        if ( instance == null) {
            instance = new Elevator();
        }
        return instance;
        // if an instance of an Elavator already exists, it is returned. if not, a new one is created.
    }

    public void configureSparks () {
        master.configure(defaultConfig, resetMode, persistMode);
        // configure sparkMAX motor controllers
    }
    
    public void configureDistSensor () {
        try {
            distSensor.setRangingMode(LaserCan.RangingMode.LONG);
            distSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            distSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
          } catch (ConfigurationFailedException e) {
            System.out.println("!LaserCAN config failed! " + e);
          }

          // attempts to configure LaserCAN, if the configuration fails, it prints an error message
    }
    
    public double getDistance() {
        double measurement = distSensor.getMeasurement().distance_mm;
        measurement = Math.min(measurement, ElevatorConstants.MAX_HEIGHT);
        measurement = Math.max(measurement, ElevatorConstants.MIN_HEIGHT);
        return measurement;
    }

    public void setOpenLoop (double demand) {
        master.set(demand);
        slave.set(demand);
    }

    public double getGoalSetpoint () {
        return goalPoint;
    }

    public void setGoalPoint (ElevatorSetpoint setpoint) {
        goalPoint = setpoint.getSetpoint();
    }

    public void setGoalPoint (double setpoint) {
        goalPoint = setpoint;
    }
}
