// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private static CANSparkMax clawMotor;
  /** Creates a new ExampleSubsystem. */
    public Claw() {
        clawMotor = new CANSparkMax(Constants.CLAW_ID, MotorType.kBrushless);
    }

    public void Open() {
        clawMotor.set(0.1);
    }

    public void Close() {
        clawMotor.set(-0.1);
    }
}
