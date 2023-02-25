// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Joystick;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // controls
  private Joystick joystick;
  private Joystick joystick2;

  //accelerometer
  private BuiltInAccelerometer builtInAccelerometer;
  final int accelCalibrateBtn = 2;
  final int autoBalanceBtn = 3;
  private double pitchBias = 0;
  private double gravity = -9.81;

  // drive modifiers mapping
  final int driveSpeedUpBtn = 5;
  final int driveSpeedDownBtn = 6;
  final int driveReverseBtn = 1;

  // functional button mapping
  final int funcReverseBtn = 1;
  final int openGripper = 5;
  final int closeGripper = 6;
  //final int func1Btn = 1;
  //final int func2Btn = 2;
  //final int func3Btn = 3;
  //final int func4Btn = 4;

  //drive motors and control objects
  private CANSparkMax moveMotorID5;
  private CANSparkMax moveMotorID7;
  private MotorControllerGroup rightMoveMotors;
  private CANSparkMax moveMotorID6;
  private CANSparkMax moveMotorID8;
  private MotorControllerGroup leftMoveMotors;
  private DifferentialDrive differentialDrive;
  private double driveSpeed = 1;
  
  //functional motors
  private VictorSPX funcMotor1;
  private VictorSPX funcMotor2;
  private VictorSPX funcMotor3;
  private VictorSPX funcMotor4;
  private CANSparkMax funcMotor9;
  private double funcModifier = 1;
  
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    // controls
    joystick = new Joystick(0); // Controller in port 0
    joystick2 = new Joystick(1);

    //accelerometer
    builtInAccelerometer = new BuiltInAccelerometer();

    //drive motors and control objects
    moveMotorID5 = new CANSparkMax(5,MotorType.kBrushless);
    moveMotorID7 = new CANSparkMax(7,MotorType.kBrushed);
    rightMoveMotors = new MotorControllerGroup(moveMotorID5, moveMotorID7);
    moveMotorID6 = new CANSparkMax(6,MotorType.kBrushless);
    moveMotorID8 = new CANSparkMax(8,MotorType.kBrushless);
    leftMoveMotors = new MotorControllerGroup(moveMotorID6, moveMotorID8);

    differentialDrive = new DifferentialDrive(moveMotorID6, moveMotorID5);
    
    //functional motors
    funcMotor9 = new CANSparkMax(9,MotorType.kBrushless);
    funcMotor1 = new VictorSPX(1);
    funcMotor2 = new VictorSPX(2);
    funcMotor3 = new VictorSPX(3);
    funcMotor4 = new VictorSPX(4);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    //m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
      moveMotorID6.set(0.5);       
        
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // calibrate accelerometer
    // find Y value when on flat ground to determine accelerometer bias
    pitchBias = builtInAccelerometer.getY();
    gravity = builtInAccelerometer.getZ();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // direct drive controls to the drive control object
    //moveMotorID8.set(1);

    //driving modifiers
    if(joystick.getRawButtonPressed(driveReverseBtn)) driveSpeed *= -1;
    if(joystick.getRawButtonPressed(driveSpeedUpBtn) && driveSpeed < 1) driveSpeed += 0.25;
    if(joystick.getRawButtonPressed(driveSpeedUpBtn) && driveSpeed > -1) driveSpeed -= 0.25;

    // functional modifiers
    if(joystick2.getRawButtonPressed(funcReverseBtn)) funcModifier *= -1;
    
    // Arm pitch
    if (Math.abs(joystick2.getY()) <= 0.1)
    {
      funcMotor1.set(ControlMode.PercentOutput, 0);
      funcMotor2.set(ControlMode.PercentOutput, 0);
    }
    else
    {
      funcMotor1.set(ControlMode.PercentOutput, funcModifier * joystick2.getY());
      funcMotor2.set(ControlMode.PercentOutput, funcModifier * joystick2.getY());
    }

    // Arm extend
    if (Math.abs(joystick2.getThrottle()) <= 0.1)
    {
      funcMotor9.set(0);
    }
    else
    {
      funcMotor9.set(funcModifier * joystick2.getZ() * 0.25);
    }
    
    // gripper
    if (joystick2.getRawButton(openGripper)) funcMotor4.set(ControlMode.PercentOutput, funcModifier * 0.5);
    else if (joystick2.getRawButton(closeGripper)) funcMotor4.set(ControlMode.PercentOutput, funcModifier * -0.5);
    else funcMotor4.set(ControlMode.PercentOutput, 0);

    //accelerometer auto-balance
    if (joystick.getRawButton(autoBalanceBtn))
    {
      // Y is pitch
      double pitchAngle = ((builtInAccelerometer.getY()-pitchBias)/Math.abs(gravity))*90;
      //System.out.println("angle calculated: " + String.valueOf(pitchAngle));

      // pitchAngle < 0 means we need to drive backwards
      if (Math.abs(pitchAngle)>7.5)
        differentialDrive.arcadeDrive(0, 0.35*(pitchAngle/Math.abs(pitchAngle)));
      else if(Math.abs(pitchAngle)>5)
        differentialDrive.arcadeDrive(0, 0.30*(pitchAngle/Math.abs(pitchAngle)));
      //System.out.println("Driving at speed: " + String.valueOf((1.0/45.0)*(pitchAngle)*driveSpeed));
    }
    else
    {
      differentialDrive.arcadeDrive(joystick.getX() * driveSpeed, joystick.getY() * driveSpeed);
    }

    //accelerometer calibration
    if (joystick.getRawButton(accelCalibrateBtn))
    {
      // find Y value when on flat ground to determine accelerometer bias
      pitchBias = builtInAccelerometer.getY();
      gravity = builtInAccelerometer.getZ();
    }

    
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
