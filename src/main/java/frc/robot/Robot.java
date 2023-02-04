// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  // drive modifiers mapping
  const int driveSpeedUpBtn = 7;
  const int driveSpeedDownBtn = 8;
  const int driveReverseBtn = 6;

  // functional button mapping
  const int funcReverseBtn = 5;
  const int func1Btn = 1;
  const int func2Btn = 2;
  const int func3Btn = 3;
  const int func4Btn = 4;

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

    //drive motors and control objects
    moveMotorID5 = new CANSparkMax(5,MotorType.kBrushless);
    moveMotorID7 = new CANSparkMax(7,MotorType.kBrushed);
    rightMoveMotors = new MotorControllerGroup(moveMotorID5, moveMotorID7);
    moveMotorID6 = new CANSparkMax(6,MotorType.kBrushless);
    moveMotorID8 = new CANSparkMax(8,MotorType.kBrushed);
    leftMoveMotors = new MotorControllerGroup(moveMotorID6, moveMotorID8);

    differentialDrive = new DifferentialDrive(leftMoveMotors, rightMoveMotors);
    
    //functional motors
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        
          //victorSPX.set(ControlMode.PercentOutput, 50);
        
        
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // direct drive controls to the drive control object
    differentialDrive.arcadeDrive(joystick.getX() * driveSpeed, joystick.getY() * driveSpeed);

    // driving modifiers
    if(joystick.getRawButtonPressed(driveReverseBtn)) driveSpeed *= -1;
    if(joystick.getRawButtonPressed(driveSpeedUpBtn) && driveSpeed < 1) driveSpeed += 0.25;
    if(joystick.getRawButtonPressed(driveSpeedUpBtn) && driveSpeed > -1) driveSpeed -= 0.25;

    // functional modifiers
    if(joystick.getRawButtonPressed(funcReverseBtn)) funcModifier *= -1;
    
    if (joystick.getRawButton(func1Btn)) funcMotor1.set(ControlMode.PercentOutput, funcModifier * 0.5);
    else funcMotor1.set(ControlMode.PercentOutput, 0);

    if (joystick.getRawButton(func2Btn)) funcMotor2.set(ControlMode.PercentOutput, funcModifier * 0.5);
    else funcMotor2.set(ControlMode.PercentOutput, 0);

    if (joystick.getRawButton(func3Btn)) funcMotor3.set(ControlMode.PercentOutput, funcModifier * 0.5);
    else funcMotor3.set(ControlMode.PercentOutput, 0);

    if (joystick.getRawButton(func4Btn)) funcMotor4.set(ControlMode.PercentOutput, funcModifier * 0.5);
    else funcMotor4.set(ControlMode.PercentOutput, 0);


    
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
