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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.buttons.Button;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.sim.PhysicsSim;


public class Kevin extends TimedRobot {
    /*
     * --- [1] Update CAN Device IDs and use WPI_VictorSPX where necessary ------
     */
    WPI_TalonSRX _rghtFront = new WPI_TalonSRX(12);
    WPI_TalonSRX _rghtFollower = new WPI_TalonSRX(13);
    WPI_TalonSRX _leftFront = new WPI_TalonSRX(15);
    WPI_TalonSRX _leftFollower = new WPI_TalonSRX(14);

    DifferentialDrive _diffDrive = new DifferentialDrive(_leftFront, _rghtFront);

    Joystick _joystick = new Joystick(0);
    //Button btn8 = new Button(8); 
    //TODO: Figure out Button inputs later

    Faults _faults_L = new Faults();
    Faults _faults_R = new Faults();

    @Override
    public void simulationInit() {
        PhysicsSim.getInstance().addTalonSRX(_rghtFront, 0.75, 4000, true);
        PhysicsSim.getInstance().addTalonSRX(_leftFront, 0.75, 4000, true);
        PhysicsSim.getInstance().addTalonSRX(_rghtFollower, 0.75, 4000);
        PhysicsSim.getInstance().addTalonSRX(_leftFollower, 0.75, 4000);
    }
    @Override
    public void simulationPeriodic() {
        PhysicsSim.getInstance().run();
    }

    /* AUTONOMOUS INITIALIZE */

    @Override
    public void autonomousInit(){

        Timer time = new Timer();
        time.delay(2.0); //Startup timer

        /* NOTE: MOTOR VALUES ARE ON A SCALE BETWEEN 0 AND 1; STOP TO FULL SPEED */
        Drive(0.5, 0, 5.0, time); //Forward at half speed for 5.0 seconds
        Drive(0.4, 0.2, 5.0, time); //Forward at .4 speed +-.2 favoring right direction for 5.0 seconds

        time.reset();
    }

    /* AUTONOMOUS SUPPLEMENTARY METHODS */
    
    /**
     * Drive Method
     * double @fore = base speed forward
     * double @tur = adjustment from base speed; tur > 0 = right, tur < 0 = left
     * double @duration = duration of timer
     * Timer @timer = timer object being used
     */
    public void Drive(double fore, double tur, double duration, Timer timer){
        timer.reset();
        _rghtFront.Set(fore - tur);
        _leftFront.Set(fore + tur);
        timer.delay(duration);
    }

    /* TELE-OPERATION PERIODIC */

    @Override
    public void teleopPeriodic() {

        //Initialization
        String work = "";
        boolean controller = true; //true = Extreme 3D, false = Logitech Dual
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
        if (Math.abs(forw) < 0.10) {
            forw = 0;
        }
        if (Math.abs(turn) < 0.10) {
            turn = 0;
        }

        /* drive robot */
        _diffDrive.arcadeDrive(forw, turn);

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

        /*
         * [4] adjust sensor phase so sensor moves positive when Talon LEDs are green
         */
        _rghtFront.setSensorPhase(true);
        _leftFront.setSensorPhase(true);

        /*
         * WPI drivetrain classes defaultly assume left and right are opposite. call
         * this so we can apply + to both sides when moving forward. DO NOT CHANGE
         */
        _diffDrive.setRightSideInverted(false);
    }
}


