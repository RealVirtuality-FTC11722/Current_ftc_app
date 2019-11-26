package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by Lyesome on 2018-01-13.
 * This Class contains all the methods from controlling the drive system using mecanum wheels
 */

public class Builder {
    public CRServo Paddle = null;
    public boolean onState = false;



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


    public void BuilderControl(LinearOpMode op, boolean joeButton, double joeStick) {
        if (joeButton) {
            Paddle.setPower(-0.8);
        } else {
            Paddle.setPower(Range.clip(joeStick, -0.8, 0.8));
        }
        op.telemetry.addData("Paddle: ", Paddle.getPower());
        op.telemetry.update();
    }
}