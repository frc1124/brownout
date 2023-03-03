// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PIDArm;
import frc.robot.commands.TankCommandGroup;
import frc.robot.subsystems.PIDDrive;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Stabilizer;

import java.util.HashMap;
import edu.wpi.first.wpilibj.Encoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.CounterBase;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static XboxController j;
  // The robot's subsystems and commands are defined here...

  //motors
  public final CANSparkMax leftLeader = new CANSparkMax(Constants.LEFTFRONT, MotorType.kBrushless);
  public final CANSparkMax leftFollower = new CANSparkMax(Constants.LEFTBACK, MotorType.kBrushless);
  public final CANSparkMax rightLeader = new CANSparkMax(Constants.RIGHTFRONT, MotorType.kBrushless);
  public final CANSparkMax rightFollower = new CANSparkMax(Constants.RIGHTBACK, MotorType.kBrushless);

  //encoders
  public final Encoder leftEncoder = new Encoder(4, 5, false, CounterBase.EncodingType.k4X);
  public final Encoder rightEncoder = new Encoder(2, 3, true, CounterBase.EncodingType.k4X);

  //motorcontrollers
  public final MotorControllerGroup lefts = new MotorControllerGroup(leftLeader, leftFollower);
  public final MotorControllerGroup rights = new MotorControllerGroup(rightLeader, rightFollower);

  //pidcontrollers
  public final PIDController leftDController = new PIDController(
    Constants.DIST_L_P, Constants.DIST_L_I, Constants.DIST_L_D);
  public final PIDController rightDController = new PIDController(
    Constants.DIST_R_P, Constants.DIST_R_I, Constants.DIST_R_D);
  public final PIDController leftVController = new PIDController(
    Constants.VEL_L_P, Constants.VEL_L_I, Constants.VEL_L_D);
  public final PIDController rightVController = new PIDController(
    Constants.VEL_R_P, Constants.VEL_R_I, Constants.VEL_R_D);
  //public final PIDArm armController = new PIDArm(leftFollower, leftEncoder, leftDController)

  
  public final PIDDrive left = new PIDDrive(lefts, leftEncoder, leftVController, leftDController, false);
  public final PIDDrive right = new PIDDrive(rights, rightEncoder, rightVController, rightDController, true);

  // Pneumatics
  public Pneumatics pneumatics = new Pneumatics();

  public Stabilizer stabilizer = new Stabilizer();
  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    j = new XboxController(0);
    // Configure the trigger bindings
    configureBindings();
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  public static HashMap<String, JoystickButton> logitechMap = new HashMap<String, JoystickButton>();

  public static JoystickButton getKey(String key) { 

    logitechMap.put("trigger", new JoystickButton(j, 1));
    logitechMap.put("thumb button", new JoystickButton(j, 2));
    logitechMap.put("bottom left", new JoystickButton(j, 3));
    logitechMap.put("bottom right", new JoystickButton(j, 4));
    logitechMap.put("top left", new JoystickButton(j, 5));
    logitechMap.put("top right", new JoystickButton(j, 6));
    logitechMap.put("seven", new JoystickButton(j, 7));
    logitechMap.put("eight", new JoystickButton(j, 8));
    logitechMap.put("nine", new JoystickButton(j, 9));
    logitechMap.put("ten", new JoystickButton(j, 10));
    logitechMap.put("eleven", new JoystickButton(j, 11));
    logitechMap.put("twelve", new JoystickButton(j, 12));

    return logitechMap.get(key);
  }
  
  private void configureBindings() {
    //getKey("botton right").whileHeld(new El_down(lift, Constants.Lift_BOTTOM_POINT)); (sample)

  }

  public Command getAutonomousCommand() {
    //An example command will be run in autonomous
    return (new TankCommandGroup(0, 0, Robot.rc));
  }
}
