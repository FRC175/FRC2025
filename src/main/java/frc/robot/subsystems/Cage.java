package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Cage extends SubsystemBase {
    private static Cage instance;
    private final DigitalInput hallSensor;
      // Compressor connected to a PCM with a default CAN ID (0)
    private final Compressor m_compressor;
   
    public Cage () {
        this.hallSensor = new DigitalInput(0);
        m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

        
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
        m_compressor.enableDigital();
    }

    
}
