package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.TankCommandGroup;


public class Stabilizer extends SubsystemBase {
    
<<<<<<< Updated upstream
    public final AHRS navx;
    
=======
    private final AHRS navx;
    private double pitch;
>>>>>>> Stashed changes

    public Stabilizer() {
        navx = new AHRS();
    }

<<<<<<< Updated upstream
    public TankCommandGroup stabilize() {
        while (navx.getPitch() < 0 ) {
            return new TankCommandGroup(1, 1, Robot.rc);
        } 
        while (navx.getPitch() > 180) {
            return new TankCommandGroup(-1, -1, Robot.rc);
=======
    public void stabilize() {
        pitch = navx.getPitch();
        while (pitch > 0.9 ) {
            CommandScheduler.getInstance().schedule(new TankCommandGroup(.625 * pitch, .625 * pitch, Robot.rc));
        } 
        while (pitch < -0.9) {
            CommandScheduler.getInstance().schedule(new TankCommandGroup(-.625* pitch, -.625 * pitch, Robot.rc));
>>>>>>> Stashed changes
        }
        return new TankCommandGroup(0, 0, Robot.rc);
    }

}
