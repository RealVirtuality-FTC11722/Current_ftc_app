package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Lyesome on 2018-01-13.
 * This file contains all the instructions for controlling the robot in Teleop mode.
 */

@TeleOp(name="Driver Mode - Only", group="Linear OpMode")  // @Autonomous(...) is the other common choice
//@Disabled
public class DriverMode extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    //create a new robot named astroGary
    private BotConfig astroGary = new BotConfig();
    //public CRServo testPaddle = null;



    @Override
    public void runOpMode() {

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        //Provide warning for drivers not to hit play until initializing is complete.
        telemetry.addData("Status", "Initializing. Please Wait...");
        telemetry.update();


        //Use the Teleop initialization method
        astroGary.InitTele(hardwareMap);
        //testPaddle = hardwareMap.get(CRServo.class, "servoPaddle");

        //Set toggle initial states
        boolean togglePressed = false;
        boolean toggleReleased = true;

        boolean ltTogglePressed = false;
        boolean ltToggleReleased = true;

        //Tell drivers that initializing is now complete
        telemetry.setAutoClear(true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //indianaGary.myRelicArm.relicGrab.setPosition(indianaGary.myRelicArm.RELIC_GRAB_OPEN);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            if (togglePressed) {
                toggleReleased = false;
            } else {
                toggleReleased = true;
            }

            //Pass controls to the drive control method.
            astroGary.drive.DriveControl(
                    BotControls.DriveYStick(this),
                    BotControls.DriveXStick(this),
                    BotControls.TurnStick(this),
                    BotControls.DriveThrottle(this));

            astroGary.Collecta.SpinnerControl(gamepad2.x, gamepad2.b, gamepad2.a);

            if (gamepad2.y && toggleReleased) {
                togglePressed = true;
                astroGary.Collecta.ChangeArmState();
            }

            astroGary.Builda.BuilderControl(this, gamepad2.right_bumper, gamepad2.right_stick_y);

            telemetry.addData("FR Wheel: ", astroGary.drive.motorFR.getPower());
            telemetry.addData("FL Wheel: ", astroGary.drive.motorFL.getPower());
            telemetry.addData("BR Wheel: ", astroGary.drive.motorBR.getPower());
            telemetry.addData("BL Wheel: ", astroGary.drive.motorBL.getPower());
            telemetry.addData("Joe's Joint: ", astroGary.Collecta.Joint.getPosition());
            if (astroGary.Collecta.ARM_UP) {
                telemetry.addData("Joint State", "Up and onwards");
            }

            if (astroGary.Collecta.ARM_DOWN) {
                telemetry.addData("Joint State", "Down and backwards");
            }
            if (astroGary.Collecta.SpinnerR.getPower() > 0) {
                telemetry.addData("Spinner", "collecting");
            }

            if (astroGary.Collecta.SpinnerR.getPower() < 0) {
                telemetry.addData("Spinner", "barfing");
            }

            if (astroGary.Collecta.SpinnerR.getPower() == 0) {
                telemetry.addData("Spinner", "standing still and doing nothing like a log");
            }

            telemetry.update();

        }

    }
}
