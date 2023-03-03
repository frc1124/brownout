// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Arm;

// public class ArmExtend extends CommandBase {
//     @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
//     private final Arm arm;
    
//     public ArmExtend(Arm arm) {
//         this.arm = arm;

//         addRequirements(arm);
//     }

//     @Override
//     public void initialize() {}

//     @Override 
//     public void execute() {
//         arm.extend();
//     }

//     @Override
//     public void end(boolean interrupted) {}

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }