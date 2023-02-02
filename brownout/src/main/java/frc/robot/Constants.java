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

  public static final int LEFTFRONT = 15; //incorrect
  public static final int RIGHTFRONT = 15; //incorrect
  public static final int LEFTBACK = 15; //incorrect
  public static final int RIGHTBACK = 15; //incorrect

  //velocity
  public static final double VEL_L_P = 1;
  public static final double VEL_L_I = 1;
  public static final double VEL_L_D = 1;

  public static final double VEL_R_P = 1;
  public static final double VEL_R_I = 1;
  public static final double VEL_R_D = 1;

  //distance
  public static final double DIST_L_P = 1;
  public static final double DIST_L_I = 1; 
  public static final double DIST_L_D = 1; 

  public static final double DIST_R_P = 1; 
  public static final double DIST_R_I = 1;  
  public static final double DIST_R_D = 1; 

  //max
  public static final double MAXSPEED = 0;
  //public static final double MAXANGULARSPEED = 0;

  public static final int WHEELRADIUS = 0;
  public static final int ENCODERRESOLUTION = 42;
  //SparkMax IDS
  public static final int CLAW_ID = 12; //not the correct id
  public static final int ARM_STAGE_ONE = 13; //not the correct id
  public static final int ARM_STAGE_TWO = 14; //not the correct id
  public static final int ARM_STAGE_THREE = 14; //not the correct id

}
