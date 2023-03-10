// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.commands.TankCommandGroup;
//import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Stabilizer;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import frc.robot.commands.Autos;
import javax.print.attribute.standard.PresentationDirection;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerCvJNI;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.commands.Stabilize;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  GenericEntry LeftVTracker;
  double velA =0;
  //CameraServer cam;
  private Command autoCMD;
  ShuffleboardTab tab;
  AnalogPotentiometer pressureTransducer;
  public static RobotContainer rc;
  String middle_auto = "middle auto";
  String auto_selected;
  private final SendableChooser<Command> chooser = new SendableChooser<>();
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code
   */
  @Override
  public void robotInit() {
    tab = Shuffleboard.getTab("SmartDashboard");
    // Command middle_auto = Autos.middleAuto();
    // Command default_auto = Autos.defaultAuto();
    
    // chooser.setDefaultOption("middle_auto", middle_auto);
    // chooser.addOption("Right Command", default_auto);
    // chooser.addOption("Left Command", default_auto);
    
    SmartDashboard.putData("Auto Choices", chooser);
    // CameraServer.startAutomaticCapture();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    rc = new RobotContainer();
    rc.navx.reset();
    rc.left.lastAngle = 0;
    rc.right.lastAngle = 0;
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    rc.navx.reset();
    autoCMD = rc.getAutonomousCommand();

    //schedule the autonomous command (example)
    if (autoCMD != null) {
      autoCMD.schedule();
    }
    //auto_selected = chooser.getSelected();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {   
    
  }

  @Override
  public void teleopInit() {
    PneumaticsControlModule pcm = new PneumaticsControlModule();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.cancel();
    // }
    // SmartDashboard.putData("Sol forward", new SolenoidFwd(rc.pneumatics));
    // SmartDashboard.putData("Sol back", new SolenoidFwd(rc.pneumatics));
    // SmartDashboard.putData("Compressor On", new CompOn(rc.pneumatics));
    // SmartDashboard.putData("Compressor Off", new CompOff(rc.pneumatics));
    // SmartDashboard.putNumber("Right", 0);
    // SmartDashboard.putNumber("Left", 0);
    rc.navx.reset();
    rc.stabilizer.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //SmartDashboard.putNumber("PSI", pressureTransducer.get());

    double velL = 100 * Math.pow(rc.j.getLeftY(),3); // Speed 
    //double velA=0;
    SmartDashboard.putNumber("Val", rc.j.getRightX());
    if (rc.j.getRightX() < 0.25 && rc.j.getRightX()> -0.25) {
      velA += 0;
    } else {
      velA += rc.j.getRightX()*360; // Angle
      SmartDashboard.putNumber("Angle", velA);
  }

    

    //LeftVTracker.setDouble(rc.rightEncoder.getRate());

    //SmartDashboard.putNumber("Right", rc.rightEncoder.getRate());
    //SmartDashboard.putNumber("Left", rc.rightEncoder.getRate());
    
    SmartDashboard.putNumber("rightjoystick (Angle)", velA);
    SmartDashboard.putNumber("leftjoystick (Speed)", velL);

    
    SmartDashboard.putNumber("navX Yaw", rc.stabilizer.getYaw());
    SmartDashboard.putNumber("navX Pitch", rc.stabilizer.getPitch());
    SmartDashboard.putNumber("navX Roll", rc.stabilizer.getRoll());
    
    CommandScheduler.getInstance().schedule(new TankCommandGroup(
    velL, 
    velL, 
    velA, 
    rc
    )); 

    //CommandScheduler.getInstance().schedule(new Stabilize(rc.stabilizer));
  }


  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
