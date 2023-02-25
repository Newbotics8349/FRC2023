//2 inch wheel RADIUS

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//DEFAULT IMPORTS
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//WPILIB BUILT IN CLASS IMPORTS
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.math.filter.SlewRateLimiter;
//MOTOR CONTROLLER IMPORTS
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

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

  //JOYSTICK
  private Joystick joystick;

  //ACCELEROMETER
  private BuiltInAccelerometer builtInAccelerometer;
  final int accelCalibrateBtn = 9;
  final int autoBalanceBtn = 10;
  private double pitchBias = 0;
  private double gravity = -9.81;

  //MODIFIER BUTTON MAPPING
  final int driveSpeedUpBtn = 7;
  final int driveSpeedDownBtn = 8;
  final int driveReverseBtn = 6;

  //SUBSYSTEM MOTOR BUTTON MAPPING
  final int funcReverseBtn = 5;
  final int func1Btn = 1;
  final int func2Btn = 2;
  final int func3Btn = 3;
  final int func4Btn = 4;

  //DRIVE MOTORS AND MOTOR CONTROL GROUPS
  private CANSparkMax moveMotorID5; //RIGHT NEO
  private CANSparkMax moveMotorID7; //RIGHT OTHER MOTOR
  //private MotorControllerGroup rightMoveMotors;
  private CANSparkMax moveMotorID6; //LEFT NEO
  private CANSparkMax moveMotorID8; //LEFT OTHER MOTOR
  //private MotorControllerGroup leftMoveMotors;
  private DifferentialDrive differentialDrive;
  private double driveSpeed = 1;
  
  //SUBSYTEM MOTORS
  private VictorSPX funcMotor1;
  private VictorSPX funcMotor2;
  private VictorSPX funcMotor3;
  private VictorSPX funcMotor4;
  private double funcModifier = 1;
  
  //SLEW RATE LIMITERS
  SlewRateLimiter filter0 = new SlewRateLimiter(1); //X-DRIVE LIMITER
  SlewRateLimiter filter1 = new SlewRateLimiter(0.5); //Y-DRIVE LIMITER
  SlewRateLimiter filter2 = new SlewRateLimiter(1); //Y-DRIVE LIMITER (FOR AUTOBALANCE SUBSYSTEM)
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //JOYSTICK
    joystick = new Joystick(0); // Controller in port 0

    //ACCELEROMETER
    builtInAccelerometer = new BuiltInAccelerometer();

    ////DRIVE MOTORS AND MOTOR CONTROL GROUPS
    moveMotorID5 = new CANSparkMax(5,MotorType.kBrushless);
    moveMotorID7 = new CANSparkMax(7,MotorType.kBrushed);
    //rightMoveMotors = new MotorControllerGroup(moveMotorID5, moveMotorID7);
    moveMotorID6 = new CANSparkMax(6,MotorType.kBrushless);
    moveMotorID8 = new CANSparkMax(8,MotorType.kBrushless);
    //leftMoveMotors = new MotorControllerGroup(moveMotorID6, moveMotorID8);

    //DIFFERENTIAL DRIVE OBJECT
    differentialDrive = new DifferentialDrive(moveMotorID6, moveMotorID5);
    
    //SUBSYSTEM MOTORS
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
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() 
  {
    //STARUP ACCELEROMETER CALIBRATION
    //find Y value when on flat ground to determine accelerometer bias
    pitchBias = builtInAccelerometer.getY();
    gravity = builtInAccelerometer.getZ();
    System.out.println(pitchBias);
    System.out.println(gravity);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  
    //DRIVE VELOCITY MODIFIERS
    if(joystick.getRawButtonPressed(driveReverseBtn)) driveSpeed *= -1; //REVERSE DRIVE MOTORS
    if(joystick.getRawButtonPressed(driveSpeedUpBtn) && driveSpeed < 1) driveSpeed += 0.25; //INCREASE VELOCITY WHEN POSITIVE
    if(joystick.getRawButtonPressed(driveSpeedUpBtn) && driveSpeed > -1) driveSpeed -= 0.25; //INCREASE (DECREASE) VELOCITY WHEN NEGATIVE

    //SUBSYSTEM MODIFIERS
    if(joystick.getRawButtonPressed(funcReverseBtn)) funcModifier *= -1; //REVERSE SUBSYSTEM MOTORS
    
    //SUBSYSTEM MOTORS
    if (joystick.getRawButton(func1Btn)) funcMotor1.set(ControlMode.PercentOutput, funcModifier * 0.1); //ARM RAISE SUBSYSTEM (1)
    else funcMotor1.set(ControlMode.PercentOutput, 0);

    if (joystick.getRawButton(func2Btn)) funcMotor2.set(ControlMode.PercentOutput, funcModifier * 0.5); //ARM RAISE SUBSYSTEM (2)
    else funcMotor2.set(ControlMode.PercentOutput, 0);

    if (joystick.getRawButton(func3Btn)) funcMotor3.set(ControlMode.PercentOutput, funcModifier* joystick.getZ()); //SUBSYSTEM 3 MOTOR
    else funcMotor3.set(ControlMode.PercentOutput, 0);

    if (joystick.getRawButton(func4Btn)) funcMotor4.set(ControlMode.PercentOutput, funcModifier * 0.5); //ARM EXTEND SUBSYSTEM (4)
    else funcMotor4.set(ControlMode.PercentOutput, 0);

    //AUTOBALANCE SUBSYSTEM
    if (joystick.getRawButton(autoBalanceBtn))
    {
      double pitchAngle = ((builtInAccelerometer.getY()-pitchBias)/Math.abs(gravity))*90; //DETERMINES Y-AXIS PITCH

      if (Math.abs(pitchAngle)>7.5)
        differentialDrive.arcadeDrive(0, filter2.calculate(0.40*(pitchAngle/Math.abs(pitchAngle)))); //VELOCITY IF GREATER THAN PITCH OF 7.5
      else if(Math.abs(pitchAngle)>5)
        differentialDrive.arcadeDrive(0, filter2.calculate(0.30*(pitchAngle/Math.abs(pitchAngle)))); //VELOCITY IF GREATER THAN PITCH OF 5
    }
    else
    {
      differentialDrive.arcadeDrive(filter0.calculate(joystick.getX() * driveSpeed), filter1.calculate(joystick.getY() * driveSpeed));
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
