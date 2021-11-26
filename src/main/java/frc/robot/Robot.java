// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import edu.wpi.first.wpilibj.Talon;
//import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import javax.swing.table.TableColumn;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

//import edu.wpi.first.wpilibj.math.*;
import edu.wpi.first.wpilibj.Timer;


import java.util.ArrayList;

import com.ctre.phoenix.music.Orchestra;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
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
  private TalonSRX m_climber;
  private final Timer m_timer = new Timer();
  private NetworkTable m_ntwktbl;
  private double m_limelight_target;
  private double m_limelight_tx;
  private double m_limelight_ty;
  private double m_limelight_tz;

  @Override
  public void robotInit() {
    m_ntwktbl = NetworkTableInstance.getDefault().getTable("limelight");
   
    


  }
  @Override
  public void teleopInit() {
    m_timer.reset();
    m_timer.start();

  }
  @Override
  public void teleopPeriodic() {
    if(m_timer.get() >= 2.0)
    {
      m_limelight_target = m_ntwktbl.getEntry("tv").getDouble(0);
      m_limelight_tx = m_ntwktbl.getEntry("tx").getDouble(0);
      m_limelight_ty = m_ntwktbl.getEntry("ty").getDouble(0);
      m_limelight_tz = m_ntwktbl.getEntry("tz").getDouble(0);
      System.out.print("limelight......."+"\n");
      System.out.print(m_limelight_target);
      System.out.print("\n");
      System.out.print(m_limelight_tx);
      System.out.print("\n");
      System.out.print(m_limelight_ty);
      System.out.print("\n");
      System.out.print(m_limelight_tz);
      System.out.print("\n");
      System.out.print("............... "+"\n");
      m_timer.reset();
    }
    


  }
  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }
  @Override
  public void autonomousPeriodic() {

  }
}
