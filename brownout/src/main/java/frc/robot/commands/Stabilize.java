// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Stabilizer;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/** An example command that uses an example subsystem. */
public class Stabilize extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Stabilizer m_subsystem;
  private final RobotContainer rc = new RobotContainer();
 

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
    double pitch = rc.stabilizer.navx.getPitch()/31.25;
    if (rc.stabilizer.navx.getPitch() > 5) {
      CommandScheduler.getInstance().schedule(new TankCommandGroup(30* pitch, 30 * pitch, 0));
    } else if (rc.stabilizer.navx.getPitch() < -5) {
      CommandScheduler.getInstance().schedule(new TankCommandGroup(-30 * pitch, -30 * pitch, 0));
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
