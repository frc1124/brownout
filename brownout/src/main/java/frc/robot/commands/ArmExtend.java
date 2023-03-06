// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.subsystems.Arm;

// public class ArmExtend extends PIDCommand {
//     @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
//     private Arm arm;
    
//     public ArmExtend(double angle, PIDController controllerD, Arm arm) {
//         super(controllerD, arm::getMeasurementD, angle, output -> arm.useOutput(output, angle));
//         addRequirements(arm);
//     }

//     @Override
//     public void initialize() {}

//     // @Override 
//     // public void execute() {
//     //     arm.extend();
//     // }

//     @Override
//     public void end(boolean interrupted) {}

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
