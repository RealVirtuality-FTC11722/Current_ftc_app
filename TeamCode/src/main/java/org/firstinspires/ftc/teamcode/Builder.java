package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by Lyesome on 2018-01-13.
 * This Class contains all the methods from controlling the building system using servos
 */

public class Builder {
    public CRServo Paddle = null;
    public boolean onState = false;
    //public Servo ShifterServo = null;
    public Servo FoundationGrabberA;
    public Servo FoundationGrabberB;

    double GRAB = 0.6;
    double RELEASE = 0.3;


    HardwareMap myHWMap;

    public Builder(){ //constructor
    }

    public void init(HardwareMap myNewHWMap){
        myHWMap = myNewHWMap;

        //Initialize wheel motors
        Paddle  = myHWMap.crservo.get("servoPaddle");
        FoundationGrabberA = myHWMap.servo.get("FoundationGrabberA");
        FoundationGrabberB = myHWMap.servo.get("FoundationGrabberB");
        //ShifterServo = myHWMap.servo.get("servoShifter");


        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery

        Paddle.setDirection(CRServo.Direction.FORWARD);
        FoundationGrabberA.setDirection(Servo.Direction.FORWARD);
        FoundationGrabberB.setDirection(Servo.Direction.FORWARD);
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

    public void BuilderControl (LinearOpMode op, boolean ShifterP, boolean ShifterN) {
        if (ShifterP) {
            //ShifterServo.setPosition(+0.799);
        }

        if (ShifterN) {
            //ShifterServo.setPosition(-0.1);


        }

    }
    }