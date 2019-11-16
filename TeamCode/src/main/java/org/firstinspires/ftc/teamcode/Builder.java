package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Lyesome on 2018-01-13.
 * This Class contains all the methods from controlling the drive system using mecanum wheels
 */

public class Builder {
    public CRServo Paddle = null;



    HardwareMap myHWMap;

    public Builder(){ //constructor
    }

    public void init(HardwareMap myNewHWMap){
        myHWMap = myNewHWMap;

        //Initialize wheel motors
        Paddle  = myHWMap.crservo.get("servoPaddle");


        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery

        Paddle.setDirection(CRServo.Direction.FORWARD);

        Paddle.setPower(0);

    }


    public void BuilderControl(LinearOpMode op, boolean joeButton, double joeStick){
        ElapsedTime paddleTime = new ElapsedTime();
        double SPIN_TIME = 2000;
        if (joeButton){
            paddleTime.reset();
            while (op.opModeIsActive() || paddleTime.time() < SPIN_TIME){
                Paddle.setPower(1);
            }
        }
        Paddle.setPower(joeStick);

    }
}