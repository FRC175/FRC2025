package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Cage extends SubsystemBase {
    private static Cage instance;
    private final DigitalInput hallSensor;
    private final PneumaticHub pcm;
    private final Compressor compressor;
    private final DoubleSolenoid cageSolenoid, funnelPin;
   
    public Cage() {
        this.hallSensor = new DigitalInput(0);
        this.pcm = new PneumaticHub(22);
        this.compressor = pcm.makeCompressor();
        compressor.disable();
        this.cageSolenoid = pcm.makeDoubleSolenoid(2,3);
        this.funnelPin = pcm.makeDoubleSolenoid(0,1);
        cageSolenoid.set(Value.kOff);
        for (int i =0; i <4; i++) {
        pcm.setOneShotDuration(i, 1000);
        }
    }

    public boolean getSensor() {
        return hallSensor.get();
    }

    @Override
    public void periodic() {
      
    }

    public static Cage getInstance() {
       if (instance == null) {
        instance = new Cage();
       }
       return instance;
    }

    public void enableCompressor() {
        compressor.enableDigital();
    }

    public void enableCage () {
       cageSolenoid.set(Value.kForward);
       // during testing, amke sure that solenoid stays in this position
    }
     public void collapseFunnel() {
        funnelPin.set(Value.kForward);
     }

   
    
}
