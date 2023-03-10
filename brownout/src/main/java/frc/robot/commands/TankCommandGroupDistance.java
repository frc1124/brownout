package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;


public class TankCommandGroupDistance extends ParallelCommandGroup{
    private RobotContainer rc;
    private double distance;
    public TankCommandGroupDistance(double speed, double distance, RobotContainer rc) {
        super();
        this.rc = rc;
        this.distance = distance;

        addCommands((Command) new TankDistance(speed, distance, rc.left));
        addCommands((Command) new TankDistance(speed, distance, rc.right));
    }
}