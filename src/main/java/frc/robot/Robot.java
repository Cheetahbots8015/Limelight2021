// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.*;
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
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.math.*;
import edu.wpi.first.wpilibj.Timer;


import java.util.ArrayList;

import com.ctre.phoenix.music.Orchestra;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import java.lang.*;
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
  private TalonFX m_testSwerve;
  private NetworkTable m_ntwktbl;
  private double m_limelight_target;
  private double m_limelight_tx;
  private double m_limelight_ty;
  private double m_limelight_tz;
  private double m_limelight_tx0;
  private double m_limelight_ty0;
  private float KpAim = -0.1f;
  private float KpDistance = -0.1f;
  private float min_aim_command = 0.05f;
  private float right_command=0.0f;
  private float left_command=0.0f;
  private float angle_of_limelight = 15.0f; 
  private float height_of_limelight = 45.0f;
  private float height_of_target = 66.0f;
  private float distance_stop = -10000000000f;
  
    /* The orchestra object that holds all the instruments */
  Orchestra _orchestra;

    /* Talon FXs to play music through.  
    More complex music MIDIs will contain several tracks, requiring multiple instruments.  */
    TalonFX [] _fxes =  { new TalonFX(18), new TalonFX(19) };

    /* An array of songs that are available to be played, can you guess the song/artists? */
  String[] _songs = new String[] {
    "song1.chrp",
    "song2.chrp",
    "song3.chrp",
    "song4.chrp",
    "song5.chrp",
    "song6.chrp",
    "song7.chrp",
    "song8.chrp",
    "song9.chrp", /* the remaining songs play better with three or more FXs */
  };

    /* track which song is selected for play */
  int _songSelection = 0;
      /* overlapped actions */
  int _timeToPlayLoops = 0;

  int _lastButton = 0;
  int _lastPOV = 0;
  
  int getButton() {
    for (int i = 1; i < 9; ++i) {
        if (m_leftStick.getRawButton(i)) {
            return i;
        }
    }
    return 0;
}

  void LoadMusicSelection(int offset)
  {
    /* increment song selection */
    _songSelection += offset;
    /* wrap song index in case it exceeds boundary */
    if (_songSelection >= _songs.length) {
        _songSelection = 0;
    }
    if (_songSelection < 0) {
        _songSelection = _songs.length - 1;
    }
    /* load the chirp file */
    _orchestra.loadMusic(_songs[_songSelection]); 

    /* print to console */
    System.out.println("Song selected is: " + _songs[_songSelection] + ".  Press left/right on d-pad to change.");
    
    /* schedule a play request, after a delay.  
        This gives the Orchestra service time to parse chirp file.
        If play() is called immedietely after, you may get an invalid action error code. */
    _timeToPlayLoops = 10;
  }


  @Override
  public void robotInit() {
    m_ntwktbl = NetworkTableInstance.getDefault().getTable("limelight");
   
    m_LeftFrontMotor = new CANSparkMax(2, MotorType.kBrushless);
    m_LeftRearMotor = new CANSparkMax(3, MotorType.kBrushless);
    m_RightFrontMotor = new CANSparkMax(1, MotorType.kBrushless);
    m_RightRearMotor = new CANSparkMax(4, MotorType.kBrushless);

    m_upperwheel = new CANSparkMax(5, MotorType.kBrushless);
    m_lowerwheel = new CANSparkMax(6, MotorType.kBrushless);


    m_roller = new CANSparkMax(7,MotorType.kBrushless);
    m_transporter =new TalonSRX(15);
    m_intake = new TalonSRX(16);
    m_testSwerve=new TalonFX(14);

    m_LeftMotor = new SpeedControllerGroup(m_LeftFrontMotor, m_LeftRearMotor);
    m_RightMotor = new SpeedControllerGroup(m_RightFrontMotor, m_RightRearMotor);
    m_climber = new TalonSRX(12);

    m_myRobot = new DifferentialDrive(m_LeftMotor, m_RightMotor);


    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
    
    /* A list of TalonFX's that are to be used as instruments */
    ArrayList<TalonFX> _instruments = new ArrayList<TalonFX>();
      
     /* Initialize the TalonFX's to be used */
    for (int i = 0; i < _fxes.length; ++i) {
        _instruments.add(_fxes[i]);
    }
    /* Create the orchestra with the TalonFX instruments */
    _orchestra = new Orchestra(_instruments);

  }
  @Override
  public void teleopInit() {
      
      /* load whatever file is selected */
      LoadMusicSelection(0);
  }
  @Override
  public void teleopPeriodic() {
    _orchestra.play();
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
    if (m_leftStick.getRawButton(8)){
      m_climber.set(ControlMode.PercentOutput, 0.2);
    }else if (m_leftStick.getRawButton(9)){
      m_climber.set(ControlMode.PercentOutput, -0.2);
    }else{
      m_climber.set(ControlMode.PercentOutput, 0);
    }


    if(m_leftStick.getRawButton(2))
    {
      m_transporter.set(ControlMode.PercentOutput, 0.75);
      m_lowerwheel.set(-0.25);
      m_upperwheel.set(0.3);
    }
    else{
      m_transporter.set(ControlMode.PercentOutput, 0); 
      m_lowerwheel.set(0);
      m_upperwheel.set(0);
    }
    int currentPOV = m_leftStick.getPOV();

      /* if song selection changed, auto-play it */
      if (_timeToPlayLoops > 0) {
        --_timeToPlayLoops;
        if (_timeToPlayLoops == 0) {
            /* scheduled play request */
            System.out.println("Auto-playing song.");
            _orchestra.play();
        }
    }

      /* has POV/D-pad changed? */
      if (_lastPOV != currentPOV) {
        _lastPOV = currentPOV;

      switch (currentPOV) {
        case 90:
        /* increment song selection */
          LoadMusicSelection(+1);
          break;
        case 270:
          /* decrement song selection */
          LoadMusicSelection(-1);
          break;
        }
      }

  }
  
  @Override
  public void autonomousInit() {
  m_timer.reset();
  m_timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    if (m_timer.get() > 0.1){
      m_limelight_target = m_ntwktbl.getEntry("tv").getDouble(0);
      m_limelight_tx = m_ntwktbl.getEntry("tx").getDouble(0);
      m_limelight_ty = m_ntwktbl.getEntry("ty").getDouble(0);
      m_limelight_tx0 = m_ntwktbl.getEntry("tx0").getDouble(0);
      m_limelight_ty0 = m_ntwktbl.getEntry("ty0").getDouble(0);
      m_limelight_tz = m_ntwktbl.getEntry("tz").getDouble(0);

      if (m_limelight_target != 1.0){
        float steering_adjust = 0.3f;

        float left_command = steering_adjust;
        float right_command = -steering_adjust;

        //m_myRobot.tankDrive(left_command,right_command);

        m_timer.reset();
      }

      else{
        float distance = (float) ((height_of_target - height_of_limelight) / Math.tan(Math.toRadians(m_limelight_ty + angle_of_limelight)));

        System.out.print(distance + "\n" );

        if (distance < distance_stop){
          m_myRobot.tankDrive(0,0);

          m_timer.reset();
          }
        else if (distance > distance_stop){
       
          double heading_error = -m_limelight_tx0;
          double distance_error = -m_limelight_ty0;
          double steering_adjust = 0.0f;
  
          if (m_limelight_tx0 > 1.0){
            steering_adjust = KpAim * heading_error - min_aim_command;
            }
          else if (m_limelight_tx0 < 1.0){
            steering_adjust= KpAim * heading_error + min_aim_command;
            }
          double distance_adjust = KpDistance * distance_error;
  
          left_command = (float) (steering_adjust + distance_adjust);
          right_command = (float) (-steering_adjust + distance_adjust);
  
          System.out.print("left and right and distance" + "\n");
          System.out.print(left_command);
          System.out.print("\n");
          System.out.print(right_command);
          System.out.print("\n");
          System.out.print(distance);
          System.out.print("\n");
          m_timer. reset();
    
          //m_myRobot.tankDrive( left_command,  right_command);
        }
      }
    }
  }
}