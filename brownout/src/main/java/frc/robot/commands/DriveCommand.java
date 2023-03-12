// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PIDDrive;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private PIDDrive leftSide;
  private PIDDrive rightSide;
 // private PIDController leftController;
//  private PIDController rightController;
// TODO Add navx
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand( PIDDrive leftSide, PIDDrive rightSide){
    this.leftSide = leftSide;
    this.rightSide = rightSide;
    addRequirements(leftSide);
    addRequirements(rightSide);
  }

  private calculateWithDeadband(double v, double deadband){
    return (v -deadband*Math.abs(v)/v)(1-deadband);
  }

  @Override
  public void execute() {
    double x = Math.pow(calculateWithDeadband(rc.j.getRightX(), 0.1),3);
    double y = Math.pow(calculateWithDeadband(rc.j.getLeftY(), 0.1),3);
   
    double a = Math.atan(y/x);
    double leftSpeed = (y == 0) ? x :y;
    double rightSpeed = (y == 0) ? x :y;
    if (x > 0){
        rightSpeed =(y > 0)  ? y - x : y + x;
    } else if ( x < 0) {
      leftSpeed = (y < 0) ? y - x : y + x;   
    }

    leftSide.useOutputV(leftSpeed, leftSpeed, a);
    rightSide.useOutputV(rightSpeed, rightSpeed, a);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    side.set(0);
  }

  //Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
