// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
    private static CANSparkMax clawMotor;
    private RelativeEncoder encoder;
  /** Creates a new ExampleSubsystem. */
    public Claw() {
        clawMotor = new CANSparkMax(Constants.CLAW_ID, MotorType.kBrushless);
        encoder = clawMotor.getEncoder();
    }

    public void Open() {
        while (true) {
            clawMotor.set(.75);
            if (encoder.getPosition() == 0.7) {
                clawMotor.set(0);
                break;
            }
        } 
    }

    public void Close() {
        while (true) {
            clawMotor.set(-.75);
            if (encoder.getPosition() == .2) {
                clawMotor.set(0);
                break;
            }
        } 
    }
}
