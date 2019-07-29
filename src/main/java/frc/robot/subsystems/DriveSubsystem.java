/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveManuallyCommand;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
public WPI_TalonSRX leftMaster = new WPI_TalonSRX(RobotMap.leftMasterPort);
public WPI_VictorSPX leftSlave = new WPI_VictorSPX(RobotMap.leftSlavePort);
public WPI_VictorSPX leftSlave2 = new WPI_VictorSPX(RobotMap.leftSlavePort2);
public WPI_TalonSRX rightMaster = new WPI_TalonSRX(RobotMap.rightMasterPort);
public WPI_VictorSPX rightSlave = new WPI_VictorSPX(RobotMap.rightSlavePort);
public WPI_VictorSPX rightSlave2 = new WPI_VictorSPX(RobotMap.rightSlavePort2);
public DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

// Hey Rishi: add that if statement later for the limelight button
public DriveSubsystem(){
  leftMaster.enableCurrentLimit(true);
  rightMaster.enableCurrentLimit(true);
  leftMaster.configContinuousCurrentLimit(15);
  leftMaster.configPeakCurrentLimit(25, 100);
  rightMaster.configContinuousCurrentLimit(15);
  rightMaster.configPeakCurrentLimit(25, 100);
  leftSlave.follow(leftMaster);
  leftSlave2.follow(leftMaster);
  rightSlave.follow(rightMaster);
  rightSlave2.follow(rightMaster);
}
public void manualDrive(double left, double right){

  //System.out.println("hohoho");
drive.tankDrive(-left,-right);
if (Math.abs(left) < 0.10){
  left = 0;
}
if (Math.abs(right) < 0.10){
  right = 0;
}




//drive.tankDrive(move,turn);
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveManuallyCommand());
  }
}
