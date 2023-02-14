package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.TankCommandGroup;


public class Stabilizer extends SubsystemBase {
    
    private final AHRS navx;
    

    public Stabilizer() {
        navx = new AHRS();
    }

    public void stabilize() {
        while (navx.getPitch() < 0 ) {
            CommandScheduler.getInstance().schedule(new TankCommandGroup(1, 1, Robot.rc));
        } 
        while (navx.getPitch() > 180) {
            CommandScheduler.getInstance().schedule(new TankCommandGroup(-1, -1, Robot.rc));
        }
    }

}
