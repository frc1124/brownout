// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase middleAuto() {
    return Commands.sequence(
      new TankCommandGroupDistance(10, 20, Robot.rc),
      new Stabilize(Robot.rc.stabilizer).withTimeout(10),
      new BrakeStop(Robot.rc.brake)
      );
  }

  public static CommandBase defaultAuto() {
    return Commands.sequence(
      new TankCommandGroupDistance(10, 30, Robot.rc)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
