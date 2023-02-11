# 8349 Java Robot Code Development Guide

## Table of Contents

[Installing Libraries](#installing-libraries)

[Motor Controllers](#motor-controllers)

[User Input](#user-input)
    

## Installing Libraries

Libraries are needed for communicating with the SparkMax and VictorSPX motor controllers.

In WPI Lib VSCode right click on build.gradle or press `Ctrl+Shift+P` and find the `Manage Vendor Libraries` option.

Select install online and paste the following links in to install the nessecary libraries:

**Victor SPX Motor Controller:** https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2023-latest.json 

**Spark MAX Motor Controller:** https://software-metadata.revrobotics.com/REVLib-2023.json


## Motor Controllers

### Victor SPX

**Setup**

To be documented.

**Initialization**

1. Include the VictorSPX libraries

    `import com.ctre.phoenix.motorcontrol.can.VictorSPX;`

    `import com.ctre.phoenix.motorcontrol.ControlMode;`
2. Define a private variable in the Robot class for each motor

    `private VictorSPX motor`
3. Create the VictorSPX object in Robot.Init()

    `motor = new VictorSPX(REPLACE_WITH_CAN_NUMBER);`

    where the CAN number is the id assigned to the motor controller

<br>

**Moving the Motor**

These motor controllers will take a decimal speed value between -1 and 1 where 1 is 100% power and -1 is 100% power in reverse.

The following is an example of moving a motor connected to a VictorSPX controller forward at 60% speed.

```lang-java
motorName.set(ControlMode.PercentOutput, 0.6);
```

### CAN Spark Max

Here is an example of moving a motor connected to a Spark Max motor controller that is connected to the CAN bus. 

```lang-java
motorName.set(0.5);
```

## User Input

### XBox Controller

1. Import the XboxController library. And Hand ID library.
```lang-java
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
```

2. Define a private variable in the Robot class for the xbox controller

```lang-java
private XboxController xboxController;
```

3. Create the Xbox Controller object in Robot.Init(). The argument of the function is the port number of the attached controller.

```lang-java
xboxController = new XboxController(0);
```

**Buttons**

 This is an example of detecting a button that is being pressed, and using that to activate and deactivate a victorSPX motor.

```lang-java
if(xboxController.getAButton())
    { intake.set(VictorSPXControlMode.PercentOutput, 0.50);
    }
else
    { intake.set(VictorSPXControlMode.PercentOutput, 0);
    }
```

The A in getAButton can be replaced with the other letters on the Xbox Controller to detect the other buttons. 

**Axis**

To use the axis associated with the left and right sticks on the controller:

```lang-java
xboxController.getY(GenericHID.Hand.kLeft)
xboxController.getX(GenericHID.Hand.kLeft)
```

getY and getX are for the y and x axis of the stick respectively. GenericHID.Hand.kLeft specifies which stick is being addressed, the left or the right. If you want to use the right stick instead of the left stick, change .kLeft to .kRight

### Flight Stick Controller

**Initialization**

1. Include the Joystick object class

    `include edu.wpi.first.wpilibj.Joystick`
2. Define a private variable in the Robot class

    `private Joystick joystick`
3. Create the Joystick object in Robot.Init()

    `joystick = new Joystick(REPLACE_WITH_PORT);`

    where the port is 0 if it is the first controller plugged in

<br>

**Button Input**

Buttons are numbered 1-16.  All button numbers are labelled on the flight stick with the exception of button 1, the trigger.

To check if a button is being pressed down use:
`joystick.getRawButton(REPLACE_WITH_BUTTON_NUMBER)`

This will return true when the button is being held down.  `joystick.getRawButtonPressed()` will only return true on the leading edge of the signal (when there is a change of state from unpressed to pressed) and `joystick.getRawButtonReleased()` will only return true on the signals' falling edge.


The following is an example of using the joystick's trigger to run a VictorSPX Motor Controller at 50% power.

```lang-java
if(joystick.getRawButton(1))
{
    victorSPX.set(ControlMode.PercentOutput, 50);
}
else
{
    victorSPX.set(ControlMode.PercentOutput, 0);
}
```

**Axis Input**

