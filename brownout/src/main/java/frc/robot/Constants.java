// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {

    public static final int kDriverControllerPort = 0;

  }
  

  public static final int LEFTFRONT = 1; 
  public static final int RIGHTFRONT = 3; 
  public static final int LEFTBACK = 2; 
  public static final int RIGHTBACK = 4; 

  //velocity
  public static final double VEL_L_P = 0.0339;// 0.037 | 0.035 - 0.036
  public static final double VEL_L_I = 0.00;
  public static final double VEL_L_D = 0.00;

  public static final double VEL_R_P = 0.0352; // 0.09
  public static final double VEL_R_I = 0.00;
  public static final double VEL_R_D = 0.00;

  //distance
  public static final double DIST_L_P = 1;
  public static final double DIST_L_I = 1; 
  public static final double DIST_L_D = 1; 

  public static final double DIST_R_P = 1; 
  public static final double DIST_R_I = 1;  
  public static final double DIST_R_D = 1; 

  // Angle
  public static final double ANGLE_L_P = 0.1;
  public static final double ANGLE_R_P = 0.5;

  // Arm
  public static final int ARM_ID = 6; // change
  
  public static final double ARM_P = 0.01;
  public static final double ARM_I = 0.01;
  public static final double ARM_D = 0.01;

  
  //max
  public static final double MAXSPEED = 0;
  //public static final double MAXANGULARSPEED = 0;

  public static final int WHEELRADIUS = 2;
  public static final int ENCODERRESOLUTION = 4000; //i think this is wrong
  //SparkMax IDS
  public static final int CLAW_ID = 5; //not the correct id

  public static final int LEFT_BRAKE = 8;
  public static final int RIGHT_BRAKE = 7;

  public static final int LEFT_SERVO = 0;
  public static final int RIGHT_SERVO = 1;

  public static final double DRIVEkS = 0;
  public static final double DRIVEkV = 0;
  public static final double kMaxAcceleration = 0; //radians per second
  public static final double kMaxVelocity = 0; // radians per second squared

}
