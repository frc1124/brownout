package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;


public class TankCommandGroup extends ParallelCommandGroup{
    private RobotContainer rc;
    private double leftV;
    private double rightV;
    public TankCommandGroup(double leftV, double rightV, double setAngle, RobotContainer rc) {
        super();
        this.rc = rc;
        this.leftV = leftV;
        this.rightV = rightV;

        addCommands((Command) new Tank(this.rightV, this.rc.rightVController, this.rc.right, setAngle));
        addCommands((Command) new Tank(this.leftV, this.rc.leftVController, this.rc.left, setAngle));
    }
}