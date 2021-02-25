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
 * Enable robot and slowly drive forward.
 * [1] If DS reports errors, adjust CAN IDs and firmware update.
 * [2] If motors are spinning incorrectly, first check gamepad (hold down btn1)
 * [3] If motors are still spinning incorrectly, correct motor inverts.
 * [4] Now that motors are driving correctly, check sensor phase.  If sensor is out of phase, adjust sensor phase.
 * [4] Is only necessary if you have sensors.
 */
package frc.robot;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.SpeedController;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
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

import edu.wpi.first.wpilibj.Timer;

import frc.robot.sim.PhysicsSim;

public class Robot extends TimedRobot {
    /*
     * --- [1] Update TalonFX motors ------
     */
    TalonFX _rghtFront = new TalonFX(12);  //right front
	TalonFX _leftFront = new TalonFX(15);
	TalonFX _leftFollower = new TalonFX(14);
	TalonFX _rghtFollower = new TalonFX(13);


    Joystick _joystick = new Joystick(0);
    //Button btn8 = new Button(8); 
    //TODO: Figure out Button inputs later

    Faults _faults_L = new Faults();
    Faults _faults_R = new Faults();

    //Autonomous variables
    double position = 0;


    /**
     * AUTONOMOUS INIT
     */

    @Override
    public void autonomousInit(){

        //Reset Encoder
        resEnc();

        /* NOTE: MOTOR VALUES ARE ON A SCALE BETWEEN 0 AND 1; STOP TO FULL SPEED */
        /* REFER TO BELOW DRIVE() METHOD FOR ARGUMENTS */
        Drive(0.4, 0.4, 15000); 
        Drive(0.25, 0.55, 15000);//110000
        System.out.println("finished");


        //Stop all motors
        _rghtFront.set(ControlMode.PercentOutput, 0);
        _leftFront.set(ControlMode.PercentOutput, 0);
    }

    /**
     * DISABLED INIT (for autonomous ease)
     */

     @Override
     public void disabledInit(){
        resEnc();
     }
    
    /**
     * DRIVE METHOD (AUTONOMOUS SUPPLEMENT)
     * double @fore = base speed forward
     * double @tur = adjustment from base speed; tur > 0 = right, tur < 0 = left
     * double @dur = encoder limit
     */
    public void Drive(double right, double left, double dur){

        //Do not proceed until position is past the duration
        while(Math.abs(position) < dur){

            _rghtFront.set(ControlMode.PercentOutput, right);
            _leftFront.set(ControlMode.PercentOutput, left);

            //Update the encoder position
            position = _leftFront.getSelectedSensorPosition(0);
            System.out.println(position);
        }

        //Reset Encoder
        resEnc();
    }

    public void resEnc(){
        _leftFront.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeoutMs);
        position = _leftFront.getSelectedSensorPosition(0);
    }

    /**
     * TELE-OPERATION PERIODIC
     */

    @Override
    public void teleopPeriodic() {

        //Initialization
        String work = "";
        double forw;
        double turn;
        double senstvty;

        /* Extreme 3D Values */
        senstvty = (-1 * _joystick.getRawAxis(3) + 1) / 2;
        forw = -1 * senstvty * _joystick.getRawAxis(1); /* positive is forward */
        turn = +1 * senstvty * _joystick.getRawAxis(2);

        /* Logitech Dual Values */
        // forw = -1 * (0.5) * _joystick.getRawAxis(1); /* positive is forward */
        // turn = +1 * (0.5) * _joystick.getRawAxis(0);
        
        System.out.println(forw);

        /* deadband gamepad 10% */
        if (Math.abs(forw) < 0.1) {
            forw = 0;
        }
        if (Math.abs(turn) < 0.1) {
            turn = 0;
        }

        /* Arcade Drive Control */
        _rghtFront.set(ControlMode.PercentOutput, (forw - turn));
        _leftFront.set(ControlMode.PercentOutput, (forw + turn));

        /*
         * [2] Make sure Gamepad Forward is positive for FORWARD, and GZ is positive for
         * RIGHT
         */
        work += " GF:" + forw + " GT:" + turn;

        /* get sensor values */
        // double leftPos = _leftFront.GetSelectedSensorPosition(0);
        // double rghtPos = _rghtFront.GetSelectedSensorPosition(0);
        double leftVelUnitsPer100ms = _leftFront.getSelectedSensorVelocity(0);
        double rghtVelUnitsPer100ms = _rghtFront.getSelectedSensorVelocity(0);

        work += " L:" + leftVelUnitsPer100ms + " R:" + rghtVelUnitsPer100ms;

        /*
         * drive motor at least 25%, Talons will auto-detect if sensor is out of phase
         */
        _leftFront.getFaults(_faults_L);
        _rghtFront.getFaults(_faults_R);

        if (_faults_L.SensorOutOfPhase) {
            work += " L sensor is out of phase";
        }
        if (_faults_R.SensorOutOfPhase) {
            work += " R sensor is out of phase";
        }

        /* print to console if btn1 is held down */
        // if (btn1) {
        //     System.out.println(work);
        // }
    }

    @Override
    public void robotInit() {
        /* factory default values */
        _rghtFront.configFactoryDefault();
        _rghtFollower.configFactoryDefault();
        _leftFront.configFactoryDefault();
        _leftFollower.configFactoryDefault();

        /* set up followers */
        _rghtFollower.follow(_rghtFront);
        _leftFollower.follow(_leftFront);

        /* [3] flip values so robot moves forward when stick-forward/LEDs-green */
        _rghtFront.setInverted(true); // !< Update this
        _leftFront.setInverted(false); // !< Update this

        /*
         * set the invert of the followers to match their respective master controllers
         */
        _rghtFollower.setInverted(InvertType.FollowMaster);
        _leftFollower.setInverted(InvertType.FollowMaster);

        /* Config the sensor used for Primary PID and sensor direction */
        _rghtFront.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 
                                            Constants.kPIDLoopIdx,
											Constants.kTimeoutMs);
		_leftFront.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 
                                            Constants.kPIDLoopIdx,
				                            Constants.kTimeoutMs);
        
        /*
         * [4] adjust sensor phase so sensor moves positive when Talon LEDs are green
         */
        _rghtFront.setSensorPhase(true);
        _leftFront.setSensorPhase(true);

        /* Config the peak and nominal outputs, 12V means full */
		_rghtFront.configNominalOutputForward(0);
		_rghtFront.configNominalOutputReverse(0);
		_rghtFront.configPeakOutputForward(1);
		_rghtFront.configPeakOutputReverse(-1);
		_leftFront.configNominalOutputForward(0);
		_leftFront.configNominalOutputReverse(0);
		_leftFront.configPeakOutputForward(1);
		_leftFront.configPeakOutputReverse(-1);

        /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		_rghtFront.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_rghtFront.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_rghtFront.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		_rghtFront.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);

		_leftFront.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_leftFront.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_leftFront.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		_leftFront.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);
        
        /**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		_rghtFront.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		_leftFront.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

        /* Config the peak and nominal outputs, 12V means full */
		_rghtFront.configNominalOutputForward(0, Constants.kTimeoutMs);
		_rghtFront.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_rghtFront.configPeakOutputForward(1, Constants.kTimeoutMs);
		_rghtFront.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		_leftFront.configNominalOutputForward(0, Constants.kTimeoutMs);
		_leftFront.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_leftFront.configPeakOutputForward(1, Constants.kTimeoutMs);
		_leftFront.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    }
}

