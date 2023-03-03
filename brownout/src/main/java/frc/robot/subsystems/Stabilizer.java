package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.TankCommandGroup;


public class Stabilizer extends SubsystemBase {
    
    public final AHRS navx;
    private double pitch;

    public Stabilizer() {
        navx = new AHRS();
    }

    public void stabilize() {
        pitch = navx.getPitch();
        while (pitch > 0.9 ) {
            CommandScheduler.getInstance().schedule(new TankCommandGroup(.625 * pitch, .625 * pitch, Robot.rc));
        } 
        while (pitch < -0.9) {
            CommandScheduler.getInstance().schedule(new TankCommandGroup(-.625* pitch, -.625 * pitch, Robot.rc));
        }
    }

}
