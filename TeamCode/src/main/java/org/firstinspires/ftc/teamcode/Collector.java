package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Lyesome on 2018-01-13.
 * This Class contains all the methods from controlling the drive system using mecanum wheels
 */

public class Collector {
    public DcMotor SpinnerL = null;
    public DcMotor SpinnerR = null;

    double PULL_IN = 1.0;
    double SPIT_OUT = -0.3;


    HardwareMap myHWMap;

    public Collector(){ //constructor
    }

    public void init(HardwareMap myNewHWMap){
        myHWMap = myNewHWMap;

        //Initialize wheel motors
        SpinnerL  = myHWMap.dcMotor.get("motorSpinnerL");
        SpinnerR  = myHWMap.dcMotor.get("motorSpinnerR");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        SpinnerL.setDirection(DcMotorSimple.Direction.FORWARD);
        SpinnerR.setDirection(DcMotorSimple.Direction.REVERSE);
        SpinnerL.setPower(0);
        SpinnerR.setPower(0);

    }


    public void CollectorControl(boolean inButton, boolean outButton, boolean stopButton){
        if (inButton){
            SpinnerL.setPower(PULL_IN);
            SpinnerR.setPower(PULL_IN);
        }

        if (outButton){
            SpinnerL.setPower(SPIT_OUT);
            SpinnerR.setPower(SPIT_OUT);
        }

        if (stopButton){
            SpinnerL.setPower(0);
            SpinnerR.setPower(0);
        }
    }
}