/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import frc.robot.commands.PneumaticsCommand;

/**
 * Add your docs here.
 */
public class PneumaticsSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public Compressor c = new Compressor(RobotMap.compressorID);

  public PneumaticsSubsystem() {
    c.setClosedLoopControl(true);
  }

  public void open(DoubleSolenoid solenoid)  {

    
    
      solenoid.set(DoubleSolenoid.Value.kForward);
    
  

    }
    public void close(DoubleSolenoid solenoid)  {

    
    
      solenoid.set(DoubleSolenoid.Value.kReverse);
    
  

    }

  

  public void assist(boolean actuate, DoubleSolenoid solenoid) {
    if (actuate == true) {
      solenoid.set(DoubleSolenoid.Value.kForward);
      //System.out.println("hi");
    } else if (actuate == false) {
      solenoid.set(DoubleSolenoid.Value.kReverse);
      //System.out.println("ey");
    } else {
      solenoid.set(DoubleSolenoid.Value.kReverse);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new PneumaticsCommand());
  }
}
