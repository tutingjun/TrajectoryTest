/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Pin;
import frc.robot.Utility;

public class drivetrain extends SubsystemBase {

  private WPI_TalonFX rMotorMasterWPI = new WPI_TalonFX(Pin.rFalconMaster);
  private WPI_TalonFX rMotorSlaveWPI = new WPI_TalonFX(Pin.rFalconSlave);
  private WPI_TalonFX lMotorMasterWPI = new WPI_TalonFX(Pin.lFalconMaster);
  private WPI_TalonFX lMotorSlaveWPI = new WPI_TalonFX(Pin.lFalconSlave);

  private final SpeedControllerGroup leftMotors;
  private final SpeedControllerGroup rightMotors;

  private final DifferentialDriveOdometry m_odometry;

  private Solenoid lShiftGear = new Solenoid(Pin.lShiftGear);
  private Solenoid rShiftGear = new Solenoid(Pin.rShiftGear);

  private AHRS ahrs = new AHRS(SPI.Port.kMXP);

  private int gearState = 0;

  public drivetrain() {

    Utility.WPITalonFXInit(rMotorMasterWPI);
    Utility.WPITalonFXInit(lMotorMasterWPI);
    Utility.WPITalonFXInit(rMotorSlaveWPI);
    Utility.WPITalonFXInit(lMotorSlaveWPI);

    rMotorSlaveWPI.follow(rMotorMasterWPI);
    lMotorSlaveWPI.follow(lMotorMasterWPI);

    lMotorMasterWPI.setInverted(true);
    lMotorSlaveWPI.setInverted(true);
    rMotorMasterWPI.setInverted(false);
    rMotorSlaveWPI.setInverted(false);

    Utility.configWPITalonFXPID(lMotorMasterWPI,0.1097, 0.22, 0, 0, 0);
    Utility.configWPITalonFXPID(rMotorMasterWPI,0.1097, 0.22, 0, 0, 0);

    leftMotors = new SpeedControllerGroup(lMotorMasterWPI, lMotorSlaveWPI);
    rightMotors = new SpeedControllerGroup(rMotorMasterWPI, rMotorSlaveWPI);

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-getCurrentAngle()));

    SmartDashboard.putNumber("Gear value", 1);
    ahrs.reset();
  }

  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(-getCurrentAngle()),
                      Utility.rawUnitToMeter(lMotorMasterWPI.getSelectedSensorPosition()),
                      Utility.rawUnitToMeter(rMotorMasterWPI.getSelectedSensorPosition()));
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetAll();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getCurrentAngle()));
  }

  public void velocityDrive(double forward, double turn){
    double lSpeed = (forward - turn) 
                      * Constants.drivetrainTargetRPM 
                      * Constants.velocityConstantsFalcon;
    double rSpeed = (forward + turn) 
                      * Constants.drivetrainTargetRPM 
                      * Constants.velocityConstantsFalcon;
    
    lMotorMasterWPI.set(ControlMode.Velocity, lSpeed);
    rMotorMasterWPI.set(ControlMode.Velocity, rSpeed);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    lMotorMasterWPI.set(ControlMode.Velocity, Utility.MeterPerSecondTorawUnit(leftSpeed));
    rMotorMasterWPI.set(ControlMode.Velocity, Utility.MeterPerSecondTorawUnit(leftSpeed));
  }

  /**
   * change to next gear ratio through shiftgear solenoids
   */
  public void shiftGear(){
    gearState += 1;
    if (gearState % 2 == 0){
      rShiftGear.set(true);
      lShiftGear.set(true);
      SmartDashboard.putNumber("Gear value", 1);
    }else{
      rShiftGear.set(false);
      lShiftGear.set(false);
      SmartDashboard.putNumber("Gear value", 2);
    }

  }

  public int getGearState(){
    return (gearState % 2) + 1;
  }

  /**
   * rotate use pid to turn
   */
  public double getCurrentAngle(){
    return Math.IEEEremainder(ahrs.getAngle(), 360);
  }

  public void resetAll(){
    rMotorMasterWPI.setSelectedSensorPosition(0);
    lMotorMasterWPI.setSelectedSensorPosition(0);
  }




}
