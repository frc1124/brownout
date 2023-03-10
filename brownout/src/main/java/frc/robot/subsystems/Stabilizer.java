package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.TankCommandGroup;


public class Stabilizer extends SubsystemBase {
    
    private AHRS navx;
    private double pitch;

    public Stabilizer(AHRS navx) {
        this.navx = navx;
        
    }
    public void stabilize() {
        pitch = navx.getRoll();
        while (pitch > 0.1 ) {
            CommandScheduler.getInstance().schedule(new TankCommandGroup(.625 * pitch, .625 * pitch, 0, Robot.rc));
        } 
        while (pitch < -0.1) {
            CommandScheduler.getInstance().schedule(new TankCommandGroup(-.625* pitch, -.625 * pitch, 0, Robot.rc));
        }
    }

    public double getVelX() {
        return navx.getVelocityX();
    }

    public double getYaw() {
        return navx.getRate();
    }

    public double getPitch() {
        return navx.getPitch();
    }

    public double getRoll() {
        return navx.getRoll();
    }

    public void reset() {
        navx.reset();
    }
}
