// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Stabilizer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Stabilize extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Stabilizer m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Stabilize(Stabilizer subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.stabilize();
    rc.stabilizer.navx.zeroYaw();
    if (rc.stabilizer.navx.getRoll() > 0) {
      velL += 10;
    } else if (rc.stabilizer.navx.getYRoll() < 0) {
      velR -= 10;
    }

    rc.stabilizer.navx.zeroYaw();
    if (rc.stabilizer.navx.getRoll() > 0) {
      rc.velL = rc.stabilizer.navx.getRoll()*k
    } else if (rc.stabilizer.navx.getYRoll() < 0) {
      rc.velR -= 10;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
