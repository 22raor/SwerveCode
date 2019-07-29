/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.PneumaticsSubsystem;

public class PneumaticsCommand extends Command {
  public PneumaticsCommand() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.pneumaticsSubsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
 
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    

      if (Robot.oi.stick3.getRawButton(3)){
      Robot.pneumaticsSubsystem.open(RobotMap.HatchRelease);
      } 

      if(Robot.oi.stick3.getRawButton(2)){
        Robot.pneumaticsSubsystem.close(RobotMap.HatchRelease);
      }
   
      // TODO Auto-generated catch block
      
      if((Robot.oi.stick.getY() < 0.1) && (Robot.oi.stick2.getY() < 0.1) ){
    Robot.pneumaticsSubsystem.assist(Robot.oi.stick3.getRawButton(6),RobotMap.FrogAssist);
      }
  }
  

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
   // end();
  }
}
