/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 10;

    //auto
    public static final double dirvetrainGearRatio = 0;

    public static final double kTrackwidthMeters = 0;//distance between two wheels
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

	//velocity constant
	public static final double velocityConstantsFalcon = 2048.0/600.0;
	public static final double velocityConstants = 4096.0/600.0;

	//rotation
	public static final double kTurnToleranceDeg = 1;
	public static final double kTurnRateToleranceDegPerS = 10;

	//target speed
	public static final double drivetrainTargetRPM = 5000;
	public static final double conveyerOutput = 0;
	public static final double spinnerOutput = 0;
	public static final double intakeMotorOutput = 0;

	//motor safety
	public static final int kCurrentLimit = 8;
	public static final int kCurrentLimitDuration = 10;



	//Joystick
		//drivetrain
		public static final int forwardAxis = 0;
		public static final int turnAxis = 0;
		public static final int bShiftGear = 0;






}

