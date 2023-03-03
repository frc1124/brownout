package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;

 public class ArmExtend extends ParallelCommandGroup{
    private RobotContainer rc;
    private double vel;

    public ArmExtend(double vel, RobotContainer rc) {
        super(); //1234567890 --TJ
        this.rc = rc;
        this.vel = vel;

        addCommands((Command) new Tank(vel, rc.leftVController, rc.left));
    }
 }