// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.subsystems.PIDDrive;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private PIDDrive leftSide;
  private PIDDrive rightSide;
  private AHRS navx;
  private double lastAngle = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand( PIDDrive leftSide, PIDDrive rightSide, AHRS navx){
    this.leftSide = leftSide;
    this.rightSide = rightSide;
    this.navx = navx;
    addRequirements(leftSide);
    addRequirements(rightSide);
  }

  private double calculateWithDeadband(double v, double deadband){
    double out = v;
    out -= deadband*Math.abs(v)/v;
    out /= (1-deadband);
    return out;
  }

  @Override
  public void execute() {
    double x = Math.pow(calculateWithDeadband(RobotContainer.j.getRightX(), 0.1),3);
    double y = Math.pow(calculateWithDeadband(RobotContainer.j.getLeftY(), 0.1),3);
   
    double a = Math.atan(y/x);
  
    double leftSpeed = (y == 0) ? x :y;
    double rightSpeed = (y == 0) ? x :y;
    if (x > 0){
        rightSpeed = (y > 0)  ? y - x : y + x;
    } else if ( x < 0) {
      leftSpeed = (y < 0) ? y - x : y + x;   
    }

    leftSide.setSetpoint( leftSpeed);
    rightSide.setSetpoint(rightSpeed);

    this.lastAngle = this.calculateWithDeadband(this.navx.getAngle(), 0.1);
   
    if (this.lastAngle != a) {
        // Adjust turn
        double diff = a - lastAngle;
        double d = Math.sin(diff);
        if (d > 0){
          rightSpeed = (y > 0)  ? y - d : y + d;
      } else if ( d < 0) {
        leftSpeed = (y < 0) ? y - d : y + d;   
      }
  
    }
    leftSide.setSetpoint(leftSpeed);
    rightSide.setSetpoint(rightSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    leftSide.set(0);
    rightSide.set(0);
  }

  //Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
