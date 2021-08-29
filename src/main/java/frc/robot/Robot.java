// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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

  @Override
  public void robotInit() {
    m_LeftFrontMotor = new CANSparkMax(2, MotorType.kBrushless);
    m_LeftRearMotor = new CANSparkMax(3, MotorType.kBrushless);
    m_RightFrontMotor = new CANSparkMax(1, MotorType.kBrushless);
    m_RightRearMotor = new CANSparkMax(4, MotorType.kBrushless);

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
    else if(m_leftStick.getX()!=0)
    {
      double factor = 0.1;
      m_myRobot.tankDrive(factor*m_leftStick.getX(), -factor*m_leftStick.getX());
    }
  }
}
