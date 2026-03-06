package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class CompressorSubsystem {
    
    public static CompressorSubsystem s_CompressorSubsystem;

    private Compressor compressor;

    public static CompressorSubsystem getInstance() {
        if (s_CompressorSubsystem == null) {
            s_CompressorSubsystem = new CompressorSubsystem();
        }
        return s_CompressorSubsystem;
    }

    private CompressorSubsystem() {

        compressor = new Compressor(41, PneumaticsModuleType.CTREPCM);

    }



    
}
