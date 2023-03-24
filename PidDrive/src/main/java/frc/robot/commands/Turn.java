// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.PidDrive;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// /** An example command that uses an example subsystem. */
// public class Turn extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final PidDrive side;
//   private double velSetpoint;
//   private double angleSetpoint;
//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public Turn(PidDrive side, double angleSetpoint, double velSetpoint) {
//     this.side = side;
//     this.velSetpoint = velSetpoint;
//     this.angleSetpoint = angleSetpoint;

//     addRequirements(side);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     side.turn(angleSetpoint, velSetpoint);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }