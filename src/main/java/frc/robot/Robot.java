// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
//import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import javax.swing.table.TableColumn;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.math.*;
/**
 * This is a demo program showing the use of the RobotDrive class, specifically it contains the code
 * necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private SpeedController m_LeftFrontMotor;
  private SpeedController m_LeftRearMotor;
  private SpeedController m_RightFrontMotor;
  private SpeedController m_RightRearMotor;
  private SpeedControllerGroup m_LeftMotor;
  private SpeedControllerGroup m_RightMotor;
  private SpeedController m_upperwheel;
  private SpeedController m_lowerwheel;
  private TalonSRX m_intake;
  private TalonSRX m_transporter;
  private SpeedController m_roller; 
  

  @Override
  public void robotInit() {
    m_LeftFrontMotor = new CANSparkMax(2, MotorType.kBrushless);
    m_LeftRearMotor = new CANSparkMax(3, MotorType.kBrushless);
    m_RightFrontMotor = new CANSparkMax(1, MotorType.kBrushless);
    m_RightRearMotor = new CANSparkMax(4, MotorType.kBrushless);

    m_upperwheel = new CANSparkMax(5,MotorType.kBrushless);
    m_lowerwheel = new CANSparkMax(6,MotorType.kBrushless);

    m_roller = new CANSparkMax(7,MotorType.kBrushless);
    m_transporter =new TalonSRX(15);
    m_intake = new TalonSRX(16);

    m_LeftMotor = new SpeedControllerGroup(m_LeftFrontMotor, m_LeftRearMotor);
    m_RightMotor = new SpeedControllerGroup(m_RightFrontMotor, m_RightRearMotor);


    m_myRobot = new DifferentialDrive(m_LeftMotor, m_RightMotor);


    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
  }

  @Override
  public void teleopPeriodic() {
    if(m_leftStick.getY() > 0.1 || m_leftStick.getY() < -0.1)
    {
      double factor = 0.75;
      m_myRobot.tankDrive(-factor*m_leftStick.getY(),-factor* m_leftStick.getY());
    }
    else if(m_leftStick.getZ()!=0)
    {
      double factor = 0.6;
      m_myRobot.tankDrive(factor*m_leftStick.getZ(), -factor*m_leftStick.getZ());
    }
    if(m_leftStick.getRawButton(1))
    {
      m_roller.set(0.05);
    }
    else{
      m_roller.set(0); 
    }

    if(m_leftStick.getRawButton(3)){
      m_intake.set(ControlMode.PercentOutput, 0.4);
    }else if (m_leftStick.getRawButton(4)){
      m_intake.set(ControlMode.PercentOutput, -0.4);
    }else{
      m_intake.set(ControlMode.PercentOutput, 0);
    }




    if(m_leftStick.getRawButton(2))
    {
      m_transporter.set(ControlMode.PercentOutput, 0.5);
      m_lowerwheel.set(-0.3);
      m_upperwheel.set(0.3);
    }
    else{
      m_transporter.set(ControlMode.PercentOutput, 0);
      m_lowerwheel.set(0);
      m_upperwheel.set(0);
    }
    
  }
}
