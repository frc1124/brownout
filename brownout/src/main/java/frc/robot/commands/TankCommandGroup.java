package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;


public class TankCommandGroup extends ParallelCommandGroup{
    private RobotContainer rc;
    private double leftV;
    private double rightV;
    public TankCommandGroup(double leftV, double rightV, RobotContainer rc) {
        super();
        this.rc = rc;
        this.leftV = leftV;
        this.rightV = rightV;

        addCommands((Command) new Tank(rightV, rc.rightVController, rc.right));
        addCommands((Command) new Tank(leftV, rc.leftVController, rc.left));
    }
}
