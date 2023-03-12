// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.PIDDrive;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TankDistance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private PIDDrive side;
//  private PIDController controller;
  double speed;
  double distance;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TankDistance(double speed, double distance, PIDDrive side){
    this.side = side;
    this.speed = speed;
    this.distance = distance;
    addRequirements(side);
  }
  
  @Override
  public void execute() {
    side.useOutputAuto(speed, distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    side.set(0);
  }

  //Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("enc" + side.getMeasurement());
    return side.getMeasurement() >= distance;
  }
}
