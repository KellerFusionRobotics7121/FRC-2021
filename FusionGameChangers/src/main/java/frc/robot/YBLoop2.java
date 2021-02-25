/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The PositionClosedLoop example demonstrates the Position closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 * Use Percent Output Mode (Holding A and using Left Joystick) to confirm talon is driving 
 * forward (Green LED on Talon) when the position sensor is moving in the postive 
 * direction. If this is not the case, flip the boolean input in setSensorPhase().
 * 
 * Controls:
 * Button 1: When pressed, start and run Position Closed Loop on Talon
 * Button 2: When held, start and run Percent Output
 * Left Joytick Y-Axis:
 * 	+ Position Closed Loop: Servo Talon forward and reverse [-10, 10] rotations
 * 	+ Percent Ouput: Throttle Talon forward and reverse
 * 
 * Gains for Position Closed Loop may need to be adjusted in Constants.java
 * 
 * Supported Version:
 * - Talon FX: 20.2.3.0
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import  edu.wpi.first.wpilibj.SpeedControllerGroup;
import  edu.wpi.first.wpilibj.SpeedController;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


public class Robot extends TimedRobot {
    /** Hardware */
	/*
	TalonFX _rghtFront = new TalonFX(12);  //right front
	TalonFX _lftFront = new TalonFX(15);
	TalonFX _leftFollower = new TalonFX(14);
	TalonFX _rightFollower = new TalonFX(13);
	*/		
	
	WPI_TalonFX _rghtFront = new WPI_TalonFX(12);
	WPI_TalonFX _rghtFollower = new WPI_TalonFX(13);
    WPI_TalonFX _lftFront = new WPI_TalonFX(15);
	WPI_TalonFX _lftFollower = new WPI_TalonFX(14);

	WPI_TalonFX topShooter = new WPI_TalonFX(2);
    WPI_TalonFX bottomShooter = new WPI_TalonFX(3);
	VictorSPX conveyor = new VictorSPX(4);
	VictorSPX intake = new VictorSPX(7);
	VictorSPX rightWrist = new VictorSPX(10);
	VictorSPX leftWrist = new VictorSPX(11);
	
	Joystick _joy = new Joystick(0);
	


  

	//DifferentialDrive m_robotDrive = new DifferentialDrive(_lftFront,_rghtFront);

	//differential drive causing the autonomous to be glitchy
	
    /** Used to create string thoughout loop */
	StringBuilder _sb = new StringBuilder();
	int _loops = 0;
	
    /** Track button state for single press event */
	boolean _lastButton1 = false;

	/** Save the target position to servo to */
	double targetPositionRotations;

	double position = 0;
	double absPosition = 0;
	int stage1Complete= 0;
	int stage2Complete= 0;
	int stage3Complete= 0;
	int stage4Complete= 0;
	int stage5Complete= 0;
	int stage6Complete =0;
	int stage7Complete =0;
	int stage8Complete =0;
	int stage9Complete =0;
	int turnAmount= 21100;
	//int straightAmount= 70000;
	int straightAmount= 120000;



	@Override
	public void autonomousInit() {
		//zeroSensors();			// Zero Sensors

	stage1Complete= 0;
	stage2Complete=0;
	stage3Complete=0;
	stage4Complete=0;
	stage5Complete=0;
	stage6Complete =0;
	stage7Complete =0;
	stage8Complete =0;
	stage9Complete =0;

_lftFront.setNeutralMode(NeutralMode.Brake);
_rghtFront.setNeutralMode(NeutralMode.Brake);

	_lftFront.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
	 // _rghtFront.setSelectedSensorPosition(0);
	}

  public void stage1()  //drive straight
  {
	position = _lftFront.getSelectedSensorPosition(0);
	//absPosition = Math.abs(position);
	//System.out.println(position);
	//currentPositionL = _leftMaster.getSelectedSensorPosition(0);
if (position<110000)
	{
	  _rghtFront.set(ControlMode.PercentOutput, -.25);
	  _lftFront.set(ControlMode.PercentOutput, .25);
	}
	 else
	{
		stage1Complete=stage1Complete+1;
	  _rghtFront.set(ControlMode.PercentOutput, 0);
	  _lftFront.set(ControlMode.PercentOutput, 0);
	  //stageOneComplete= true;
	  if (stage1Complete>4)
		{
		_lftFront.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		}
	}

}


/*public void stage2()  //right turn
{position = _lftFront.getSelectedSensorPosition(0);
  absPosition = Math.abs(position);
  //currentPositionL = _leftMaster.getSelectedSensorPosition(0);
  if (position<turnAmount)
  {
	_rghtFront.set(ControlMode.PercentOutput, .25);  //right turn
	_lftFront.set(ControlMode.PercentOutput, .25);
  }
  else 
  {
	stage2Complete=stage2Complete+1;
  _rghtFront.set(ControlMode.PercentOutput, 0);
  _lftFront.set(ControlMode.PercentOutput, 0);
  //stageOneComplete= true;
  if (stage2Complete>4)
	{
	_lftFront.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
	}
}
}
*/
public void stage2()
{position = _lftFront.getSelectedSensorPosition(0);
	absPosition = Math.abs(position);
	//currentPositionL = _leftMaster.getSelectedSensorPosition(0);
	if (position<317000)
	{
	  _rghtFront.set(ControlMode.PercentOutput, -.25);  //right turn
	  _lftFront.set(ControlMode.PercentOutput, .55);
	}
	else 
	{
	  stage2Complete=stage2Complete+1;
	_rghtFront.set(ControlMode.PercentOutput, 0);
	_lftFront.set(ControlMode.PercentOutput, 0);
	//stageOneComplete= true;
	if (stage2Complete>4)
	  {
	  _lftFront.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
	  }
  }
  }
public void stage3()  //drive straight
{position = _lftFront.getSelectedSensorPosition(0);
  absPosition = Math.abs(position);
  //System.out.println(position);
  //currentPositionL = _leftMaster.getSelectedSensorPosition(0);
  if (position<110000)
  {
	_rghtFront.set(ControlMode.PercentOutput, -.25);
	_lftFront.set(ControlMode.PercentOutput, .25);
  }
  else
  {
	  stage3Complete=stage3Complete+1;
	_rghtFront.set(ControlMode.PercentOutput, 0);
	_lftFront.set(ControlMode.PercentOutput, 0);
	//stageOneComplete= true;
	if (stage3Complete>4)
	{
	_lftFront.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
	}
  }
}

public void stage4()  //left turn
{position = _lftFront.getSelectedSensorPosition(0);
  absPosition = Math.abs(position);
 
 
  //currentPositionL = _leftMaster.getSelectedSensorPosition(0);
  if (position<122000)
  {
	_rghtFront.set(ControlMode.PercentOutput, -.55);  //right turn
	_lftFront.set(ControlMode.PercentOutput, .25);
  }
  else 
  {
	stage4Complete=stage4Complete+1;
  _rghtFront.set(ControlMode.PercentOutput, 0);
  _lftFront.set(ControlMode.PercentOutput, 0);
  //stageOneComplete= true;
  if (stage4Complete>4)
	{
	_lftFront.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
	}
}
}
	
public void stage5()  //drive straight
{position = _lftFront.getSelectedSensorPosition(0);
  absPosition = Math.abs(position);
  //System.out.println(position);
  //currentPositionL = _leftMaster.getSelectedSensorPosition(0);
  if (position<77000)
  {
	_rghtFront.set(ControlMode.PercentOutput, -.25);
	_lftFront.set(ControlMode.PercentOutput, .25);
  }
  else
  {
	  stage5Complete=stage5Complete+1;
	_rghtFront.set(ControlMode.PercentOutput, 0);
	_lftFront.set(ControlMode.PercentOutput, 0);
	//stageOneComplete= true;
	if (stage5Complete>4)
	{
	_lftFront.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
	}
  }
}

public void stage6()  
{
	position = _lftFront.getSelectedSensorPosition(0);
	absPosition = Math.abs(position);
	//currentPositionL = _leftMaster.getSelectedSensorPosition(0);
	if (position<94000)
	{
	  _rghtFront.set(ControlMode.PercentOutput, -.55);  //right turn
	  _lftFront.set(ControlMode.PercentOutput, .25);
	}
	else 
	{
	  stage6Complete=stage6Complete+1;
	_rghtFront.set(ControlMode.PercentOutput, 0);
	_lftFront.set(ControlMode.PercentOutput, 0);
	//stageOneComplete= true;
	if (stage6Complete>4)
	  {
	  _lftFront.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
	  }
  }
}

 public void stage7()
 {position = _lftFront.getSelectedSensorPosition(0);
	absPosition = Math.abs(position);
	//System.out.println(position);
	//currentPositionL = _leftMaster.getSelectedSensorPosition(0);
	if (position<300000)
	{
	  _rghtFront.set(ControlMode.PercentOutput, -.25);
	  _lftFront.set(ControlMode.PercentOutput, .25);
	}
	else
	{
		stage7Complete=stage7Complete+1;
	  _rghtFront.set(ControlMode.PercentOutput, 0);
	  _lftFront.set(ControlMode.PercentOutput, 0);
	  //stageOneComplete= true;
	  if (stage7Complete>4)
	  {
	  _lftFront.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
	  }
	}
  }
  
  public void stage8()  //right turn
  {position = _lftFront.getSelectedSensorPosition(0);
	absPosition = Math.abs(position);
	//currentPositionL = _leftMaster.getSelectedSensorPosition(0);
	if (position<turnAmount)
	{
	  _rghtFront.set(ControlMode.PercentOutput, .25);  //right turn
	  _lftFront.set(ControlMode.PercentOutput, .25);
	}
	else 
	{
	  stage8Complete=stage8Complete+1;
	_rghtFront.set(ControlMode.PercentOutput, 0);
	_lftFront.set(ControlMode.PercentOutput, 0);
	//stageOneComplete= true;
	if (stage8Complete>4)
	  {
	  _lftFront.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
	  }
  }
  }

  public void stage9()  //drive straight
  {position = _lftFront.getSelectedSensorPosition(0);
	absPosition = Math.abs(position);
	//System.out.println(position);
	//currentPositionL = _leftMaster.getSelectedSensorPosition(0);
	if (position<160000)
	{
	  _rghtFront.set(ControlMode.PercentOutput, -.25);
	  _lftFront.set(ControlMode.PercentOutput, .25);
	}
	else 
	{
		stage9Complete=stage9Complete+1;
	  _rghtFront.set(ControlMode.PercentOutput, 0);
	  _lftFront.set(ControlMode.PercentOutput, 0);
	  //stageOneComplete= true;
	  if (stage9Complete>4)
		{
		_lftFront.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
		}
	}
}


/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		position = _lftFront.getSelectedSensorPosition(0);
		//_rghtFront.setInverted(Constants.kMotorInvert);

		System.out.println(position);
		System.out.println(stage1Complete);
		System.out.println(stage2Complete);	
		System.out.println(stage3Complete);
		System.out.println(stage4Complete);
		System.out.println(stage5Complete);
		System.out.println(stage6Complete);
		System.out.println(stage7Complete);
	
if (stage1Complete<5)
{
		stage1();
}


if ((stage1Complete>4)&&(stage2Complete<5))
{
		stage2();
}


if ((stage2Complete>4)&&(stage3Complete<5))
{
		stage3();
}

if ((stage3Complete>4)&&(stage4Complete<5))
{
		stage4();
}


if ((stage4Complete>4)&&(stage5Complete<5))
{
		stage5();
}
	 
if ((stage5Complete>4)&&(stage6Complete<5))
{
		stage6();
}
			  
if ((stage6Complete>4)&&(stage7Complete<5))
{
		stage7();
}

/*
if ((stage7Complete>4)&&(stage8Complete<5))
{
		stage8();
}
if ((stage8Complete>4)&&(stage9Complete<5))
{
		stage9();
} 

*/

}


	public void robotInit() {
		
	
		/* Zero integrated sensors on Talons */
	
		_rghtFollower.set(TalonFXControlMode.Follower,12);  //follow right front
		_lftFollower.set(TalonFXControlMode.Follower,15);  //follow right front

		/* Factory Default all hardware to prevent unexpected behaviour */
		_rghtFront.configFactoryDefault();
		_lftFront.configFactoryDefault();
		
		/* Config the sensor used for Primary PID and sensor direction */
        _rghtFront.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 
                                            Constants.kPIDLoopIdx,
											Constants.kTimeoutMs);
		_lftFront.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 
                                            Constants.kPIDLoopIdx,
				                            Constants.kTimeoutMs);

		/* Ensure sensor is positive when output is positive */
		_rghtFront.setSensorPhase(Constants.kSensorPhase);
		_lftFront.setSensorPhase(Constants.kSensorPhase);

		/**
		 * Set based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. 
		 */ 
		_rghtFront.setInverted(Constants.kMotorInvert);
		//_lftFront.setInverted(Constants.kMotorInvert);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/PercentOutput.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _rghtFront.setSensorPhase(true);

		/* Config the peak and nominal outputs, 12V means full */
		_rghtFront.configNominalOutputForward(0, Constants.kTimeoutMs);
		_rghtFront.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_rghtFront.configPeakOutputForward(1, Constants.kTimeoutMs);
		_rghtFront.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		_lftFront.configNominalOutputForward(0, Constants.kTimeoutMs);
		_lftFront.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_lftFront.configPeakOutputForward(1, Constants.kTimeoutMs);
		_lftFront.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		_rghtFront.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		_lftFront.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		_rghtFront.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_rghtFront.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_rghtFront.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		_rghtFront.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);

		_lftFront.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_lftFront.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_lftFront.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		_lftFront.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
    }

	void commonLoop() {
		/* Gamepad processing */
		double leftYstick = _joy.getY();
		boolean button1 = _joy.getRawButton(1);	// X-Button
		boolean button2 = _joy.getRawButton(2);	// A-Button

		/* Get Talon's current output percentage */
		double motorOutput = _rghtFront.getMotorOutputPercent();

		/* Deadband gamepad */
		if (Math.abs(leftYstick) < 0.10) {
			/* Within 10% of zero */
			leftYstick = 0;
		}

		/* Prepare line to print */
		_sb.append("\tout:");
		/* Cast to int to remove decimal places */
		_sb.append((int) (motorOutput * 100));
		_sb.append("%");	// Percent

		_sb.append("\tpos:");
		_sb.append(_rghtFront.getSelectedSensorPosition(0));
		_sb.append("u"); 	// Native units

		/**
		 * When button 1 is pressed, perform Position Closed Loop to selected position,
		 * indicated by Joystick position x10, [-10, 10] rotations
		 */
		if (!_lastButton1 && button1) {
			/* Position Closed Loop */

			/* 10 Rotations * 2048 u/rev in either direction */
			targetPositionRotations = leftYstick * 10.0 * 2048;
			_rghtFront.set(TalonFXControlMode.Position, targetPositionRotations);
		}

		/* When button 2 is held, just straight drive */
		if (button2) {
			/* Percent Output */

			_rghtFront.set(TalonFXControlMode.PercentOutput, leftYstick);
		}

		/* If Talon is in position closed-loop, print some more info */
		if (_rghtFront.getControlMode() == TalonFXControlMode.Position.toControlMode()) {
			/* ppend more signals to print when in speed mode. */
			_sb.append("\terr:");
			_sb.append(_rghtFront.getClosedLoopError(0));
			_sb.append("u");	// Native Units

			_sb.append("\ttrg:");
			_sb.append(targetPositionRotations);
			_sb.append("u");	/// Native Units
		}

		/**
		 * Print every ten loops, printing too much too fast is generally bad
		 * for performance.
		 */
		if (++_loops >= 10) {
			_loops = 0;
			System.out.println(_sb.toString());
		}

		/* Reset built string for next loop */
		_sb.setLength(0);
		
		/* Save button state for on press detect */
		_lastButton1 = button1;
    }
	
	void shooter()

	{
		
            topShooter.set(.5);
            bottomShooter.set(.5);
		
	}
	public void updateToggle()
    {
        if(_joy.getRawButton(2)){
            if(!togglePressed){
                toggleOn = !toggleOn;
                togglePressed = true;
            }
        }else{
            togglePressed = false;
        }
    }
	/**
	 * This function is called periodically during operator control
	 */

	boolean toggleOn = false;
	boolean togglePressed = false;
	boolean intakeToggle=false;
	
	public void teleopPeriodic() {
		//commonLoop();
		updateToggle();
		double forw = (.5*( _joy.getRawAxis(1))); /* positive is forward */
        double turn =   (.5*(_joy.getRawAxis(0))); /* positive is right */
		double leftYstick = _joy.getRawAxis(1);
		double rightXstick = _joy.getRawAxis(0);
		boolean btn1 = _joy.getRawButton(1); /* is button is down, print joystick values */
        boolean btn2 = _joy.getRawButton(2);  //x-button
		boolean btn3 = _joy.getRawButton(3);  //powershot low
		boolean btn4 = _joy.getRawButton(4);  //powerShot high
        boolean btn6 = _joy.getRawButton(6);	//shooter
		boolean btn5 = _joy.getRawButton(5);	//conveyor
		boolean btn7 = _joy.getRawButton(7);
		boolean btn8 = _joy.getRawButton(8);  //intake
		boolean btn12 = _joy.getRawButton(12);  //intake
		double wristValue = _joy.getRawAxis(3);
		boolean shooterState=false;

		
		

int direction = _joy.getPOV(0);

  /* deadband gamepad 10% */
if (Math.abs(forw) < 0.10) {
	forw = 0;
}
if (Math.abs(turn) < 0.10) {
	turn = 0;
}


if (direction == 0)
 { // DPAD UP button is pressed
	_rghtFront.set(TalonFXControlMode.PercentOutput, -.1);
	_lftFront.set(TalonFXControlMode.PercentOutput, .1);
}
 else if (direction == 180) 
 { // DPAD DOWN button is pressed
	_rghtFront.set(TalonFXControlMode.PercentOutput, .1);
	_lftFront.set(TalonFXControlMode.PercentOutput, -.1);
}
else if (direction == 90) 
 { // DPAD DOWN button is pressed
	_rghtFront.set(TalonFXControlMode.PercentOutput, .1);
	_lftFront.set(TalonFXControlMode.PercentOutput, .1);
}
else if (direction == 270) 
 { // DPAD DOWN button is pressed
	_rghtFront.set(TalonFXControlMode.PercentOutput, -.1);
	_lftFront.set(TalonFXControlMode.PercentOutput, -.1);
}
else if (direction == 45) 
 { // DPAD DOWN button is pressed
	_rghtFront.set(TalonFXControlMode.PercentOutput, .1);
	_lftFront.set(TalonFXControlMode.PercentOutput, .1);
}

	else if (direction==-1)
	{
		_rghtFront.set(TalonFXControlMode.PercentOutput, forw+turn);
		_lftFront.set(TalonFXControlMode.PercentOutput, -1*(forw-turn));
	}


	
      
		
		//m_robotDrive.arcadeDrive(forw, turn);

	
		_rghtFollower.set(TalonFXControlMode.Follower,12);  //follow right front
		_lftFollower.set(TalonFXControlMode.Follower,15);  //follow left front

	
		/*
		else
		{
			topShooter.set(ControlMode.PercentOutput,0.0);
			bottomShooter.set(ControlMode.PercentOutput,0.0);	
		}
		*/
		switch (btn6) //conveyor
		{
            case 1:
    			conveyor.set(ControlMode.PercentOutput,-0.35);
                break;
            case 0:
    			conveyor.set(ControlMode.PercentOutput,0.0);
                break;
		}
		
		
		if (btn7) //intake out
		{
			intake.set(ControlMode.PercentOutput,-0.5);
		}
		
		else if (btn8) //intake in
		{
			intake.set(ControlMode.PercentOutput,0.5);
		}
		
		else
		{
		intake.set(ControlMode.PercentOutput,0.0);
		}
		
		rightWrist.set(ControlMode.PercentOutput,wristValue);
		leftWrist.set(ControlMode.PercentOutput,-wristValue);

		
		if (toggleOn)
		{
            topShooter.set(ControlMode.PercentOutput,-0.35);
			bottomShooter.set(ControlMode.PercentOutput,-0.35);
		}
		
		 else if (btn5)  //shooters
		{
			topShooter.set(ControlMode.PercentOutput,-0.5);
			bottomShooter.set(ControlMode.PercentOutput,-0.5);
		}
		else if (btn3)  //shooters low
		{
			topShooter.set(ControlMode.PercentOutput,-.25);
			bottomShooter.set(ControlMode.PercentOutput,-.25);
		}
		 else if (btn4)  //shooters high
		{
			topShooter.set(ControlMode.PercentOutput,-0.75);
			bottomShooter.set(ControlMode.PercentOutput,-0.75);
		}

		
		else 
		{
			topShooter.set(.0);
			bottomShooter.set(.0);
		}
		
		if ((btn12)&&(intakeToggle=false)) //intake in
		{
			intake.set(ControlMode.PercentOutput,0.5);
			//intakeToggle=!intakeToggle;
		}
		

	}		
}
