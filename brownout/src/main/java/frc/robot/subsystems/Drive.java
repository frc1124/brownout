package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;


public class Drive extends SubsystemBase{
    private static DifferentialDrive drive;
    private static CANSparkMax leftFront;
    private static CANSparkMax leftBack;
    private static CANSparkMax rightFront;
    private static CANSparkMax rightBack;
    private static RelativeEncoder leftEncoder;
    private static RelativeEncoder rightEncoder;

    public AHRS navx;

    public Drive() {
        leftFront = new CANSparkMax(Constants.LEFTFRONT,  MotorType.kBrushless);        
        rightFront = new CANSparkMax(Constants.RIGHTFRONT,  MotorType.kBrushless);        
        leftBack  = new CANSparkMax(Constants.LEFTBACK, MotorType.kBrushless);
        rightBack = new CANSparkMax(Constants.RIGHTBACK, MotorType.kBrushless);

        leftEncoder = leftFront.getEncoder();
        rightEncoder = rightFront.getEncoder();

        leftFront.setInverted(true);
        leftBack.setInverted(true);

        leftBack.follow(leftFront);
        rightBack.follow(rightFront);

        //?? not sure if this is needed, but this should be the circumference of the wheel
        leftEncoder.setAverageDepth((int) (2 * 3 * Math.PI / 2048));
        rightEncoder.setAverageDepth((int) (2 * 3 * Math.PI / 2048));

        navx = new AHRS();
        drive = new DifferentialDrive(leftFront, rightBack); //why is it front then back
    }

    public void stop() {
        leftFront.set(0);
        rightFront.set(0);
    }

    public void forward() {
        leftFront.set(1);
        rightFront.set(1);
    }
    
    public void backward() {
        leftFront.set(-1);
        rightFront.set(-1);
    }

    public boolean turn(double angle) {
        angle -= (double) navx.getYaw();
        if (angle < -180)
            angle += 360;
        if (angle != 0) {
            arcadeDrive(0, angle / 360);
            return true;
        } else
            return false;
    }

    public AHRS getNavxInstance() {
        return navx;
    }

    public double getAvgDistance() {
        return (leftEncoder.getPosition()  + rightEncoder.getPosition()) / 2;
    }

    public double getAvgVelocity() {
        return (leftEncoder.getVelocity() + rightEncoder.getVelocity()) / 2;
    }

    public void arcadeDrive(double fwd, double rot) {
        drive.arcadeDrive(fwd, rot);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
