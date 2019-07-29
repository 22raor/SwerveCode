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
import frc.robot.RobotMap;
import frc.robot.commands.EverythingElseCommand;

/**
 * Add your docs here.
 */
public class EverythingElseSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_TalonSRX intake1 = new WPI_TalonSRX(RobotMap.intake1);
 
  public WPI_TalonSRX climber1 = new WPI_TalonSRX(RobotMap.climber1);
  public WPI_VictorSPX climber2 = new WPI_VictorSPX(RobotMap.climber2);
  public WPI_TalonSRX elevator = new WPI_TalonSRX(RobotMap.elevator);

public EverythingElseSubsystem() {

}
public void intake(double intakeEffort1) {intake1.set(intakeEffort1);}




public void outtake(double intakeEffort2) {intake1.set(-intakeEffort2);}
public void climber(double climberEffort) {climber1.set(climberEffort);
climber2.set(climberEffort);

}
public void elevator(double elevatorEffort) {elevator.set(elevatorEffort);}




  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
     setDefaultCommand(new EverythingElseCommand());
  }
}
