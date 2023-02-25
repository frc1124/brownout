// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
// import edu.wpi.first.wpilibj.CAN;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
// import frc.robot.Constants;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;



// public class SampleMotionProfiling extends TrapezoidProfileSubsystem {
//   public final CANSparkMax leftLeader = new CANSparkMax(Constants.LEFTFRONT, MotorType.kBrushless);
//   public final SimpleMotorFeedforward lFeedForward = new SimpleMotorFeedforward(Constants.DRIVEkS, Constants.DRIVEkV);

//   /** Creates a new ExampleSubsystem. */
//   public SampleMotionProfiling() {
//     super(new TrapezoidProfile.Constraints(
//       Constants.kMaxVelocity, 
//       Constants.kMaxAcceleration));
//       //do the pid setting here 
//   }

//   @Override
//   public void useState(TrapezoidProfile.State setpoint) {
//     double feedforward = lFeedForward.calculate(setpoint.position, setpoint.velocity);
//     leftLeader.setSetpoint(CANSparkMax.PIDMode.kposition, setpoint.position, feedforward / 12.0);
//   }

//   public Command setArmGoalCommand(double kOffsetRads) {
//     return Commands.runOnce(() -> setGoal(kOffsetRads), this);
//   }

// }
