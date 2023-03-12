package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmRaise extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private Arm arm;
    
    public ArmRaise(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        arm.raise();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
