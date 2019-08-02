/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.RobotMap;
import frc.robot.commands.DriveManuallyCommand;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
public WPI_TalonSRX FLdrive = new WPI_TalonSRX(RobotMap.FLdrive);
public WPI_TalonSRX RLdrive = new WPI_TalonSRX(RobotMap.RLdrive);
public WPI_TalonSRX FRdrive = new WPI_TalonSRX(RobotMap.FRdrive);
public WPI_TalonSRX RRdrive = new WPI_TalonSRX(RobotMap.RRdrive);
public WPI_TalonSRX FLsteer = new WPI_TalonSRX(RobotMap.FLsteer);
public WPI_TalonSRX RLsteer = new WPI_TalonSRX(RobotMap.RLsteer);
public WPI_TalonSRX FRsteer = new WPI_TalonSRX(RobotMap.FRsteer);
public WPI_TalonSRX RRsteer = new WPI_TalonSRX(RobotMap.RRsteer);
Gyro gyro;






public double fwd;
public double str;
public double rcw;
public double temp;
  public double A;
  public double B;
  public double C;
  public double D;
  public double ws1, ws2, ws3, ws4;
  public double wa1, wa2, wa3, wa4, max;
  public double ang;


// Hey Rishi: add that if statement later for the limelight button
public DriveSubsystem(){

  FLsteer.configSelectedFeedbackSensor(FeedbackDevice.Analog);
  RLsteer.configSelectedFeedbackSensor(FeedbackDevice.Analog);
  FRsteer.configSelectedFeedbackSensor(FeedbackDevice.Analog);
  RRsteer.configSelectedFeedbackSensor(FeedbackDevice.Analog);

  FLdrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  RLdrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  FRdrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  RRdrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);



}


public void swerveDrive(Joystick one, Joystick two) {

ang = gyro.getAngle();



fwd = -one.getY();
str = one.getX();
rcw = two.getX();
temp =((fwd*(Math.cos(ang))) + (str*(Math.sin(ang))));
str =(((-fwd)*(Math.sin(ang))) + (str*(Math.cos(ang))));
fwd = temp;
A = (str - (rcw*(RobotMap.wheelbase/RobotMap.wtRatio))); 
B = (str + (rcw*(RobotMap.wheelbase/RobotMap.wtRatio))); 
C = (fwd - (rcw*(RobotMap.trackwidth/RobotMap.wtRatio))); 
D = (fwd + (rcw*(RobotMap.trackwidth/RobotMap.wtRatio))); 
ws1 = Math.sqrt(((B*B)+(C*C)));
ws2 = Math.sqrt(((B*B)+(D*D)));
ws3 = Math.sqrt(((A*A)+(D*D)));
ws4 = Math.sqrt(((A*A)+(C*C)));
wa1 = ((Math.atan2(B, C))*(180/Math.PI));
wa2 = ((Math.atan2(B, D))*(180/Math.PI));
wa3 = ((Math.atan2(A, D))*(180/Math.PI));
wa4 = ((Math.atan2(A, C))*(180/Math.PI));
max = ws1; 
if (ws2>max){
  max = ws2;
}
if (ws3>max){
  max = ws3;
}
if (ws4>max){
  max = ws4;
}
if(max>1){
  ws1/=max;
  ws2/=max;
  ws3/=max;
  ws4/=max;
}

FRsteer.set(ControlMode.Position,(wa1+180) * (1023/360));
FLsteer.set(ControlMode.Position,(wa2+180) * (1023/360));
RLsteer.set(ControlMode.Position,(wa3+180) * (1023/360));
RRsteer.set(ControlMode.Position,(wa4+180) * (1023/360));
if(ws1==0){ws1 =-1;} else{
  ws1 = (-1 + (2*ws1));
}
if(ws2==0){ws2 =-1;} else{
  ws2 = (-1 + (2*ws2));
}
if(ws3==0){ws3 =-1;} else{
  ws3 = (-1 + (2*ws3));
}
if(ws4==0){ws4 =-1;} else{
  ws4 = (-1 + (2*ws4));
}


FRdrive.set(ws1);
FLdrive.set(ws2);
RRdrive.set(ws3);
RLdrive.set(ws4);



}







  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveManuallyCommand());
  }
}
