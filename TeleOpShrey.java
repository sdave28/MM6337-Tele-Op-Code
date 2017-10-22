package org.firstinspires.ftc.teamcode;
/* Copyright (c) 2014 Qualcomm Technologies Inc
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior writte
+.n permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Test2", group="Competition")
public class TeleOpShrey extends OpMode {

    // TETRIX VALUES.
    final static double CLAMP_MIN_RANGE  = 0.20;
    final static double CLAMP_MAX_RANGE  = 0.7;
    final static double ARM_MAX_RANGE = 0.5;
    final static double ARM_MIN_RANGE = 0;
    final static double DISPLACER_MAX_RANGE = 0.75;

    // position of the clamp servos.
    double rightClampPosition;
    double leftClampPosition;
    double liftPosistion;
    double extenderPosition;
    double lrClampPosition;
    double rrClampPosition;

    // amount to change the arm servo position
    double clampDelta = 0.1;
    double liftDelta = 0.0125;

    //Declare Drive Train Motors
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;

    // Declare Clamp Servos
    Servo leftClamp;
    Servo rightClamp;

    //Declare Jewel Displacer Servos
    CRServo displacer;
    Servo rotator;

    //Declare lift motors
    DcMotor lift;

    //Declare balancer servo
    CRServo balancer;

    //Declare relic pick-up motors
    DcMotor extender;

    //Declare relic pick-up servos
    Servo arm1;
    Servo arm2;
    CRServo shoulderServo;
    CRServo elbowServo;

    /**
     * Constructor
     */
    public TeleOpShrey() {

    }

    @Override
    public void init() {
        //DriveTrain
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        //Glyph Clamp
        leftClamp = hardwareMap.servo.get("leftClamp");
        rightClamp = hardwareMap.servo.get("rightClamp");
        lift = hardwareMap.dcMotor.get("lift");

        //Jewel Hitter
        displacer = hardwareMap.crservo.get("displacer");
        rotator = hardwareMap.servo.get("rotator");

        //Balancer
        balancer = hardwareMap.crservo.get("balancer");

        //Relic Grabber
        arm1 = hardwareMap.servo.get("arm1");
        arm2 = hardwareMap.servo.get("arm2");
        shoulderServo = hardwareMap.crservo.get("shoulderServo");
        elbowServo = hardwareMap.crservo.get("elbowServo");
        extender = hardwareMap.dcMotor.get("extender");

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void start() {
      /*
       * Use the hardwareMap to get the dc motors and servos by name. Note
       * that the names of the devices must match the names used when you
       * configured your robot and created the configuration file.
       */

      /*
       * For the demo Tetrix K9 bot we assume the following,
       *   There are two motors "motor_1" and "motor_2"
       *   "motor_1" is on the right side of the bot.
       *   "motor_2" is on the left side of the bot.
       *
       * We also assume that there are two servos "servo_1" and "servo_6"
       *    "servo_1" controls the arm joint of the manipulator.
       *    "servo_6" controls the claw joint of the manipulator.
       */
        init();

        // assign the starting position of the wrist and claw
        leftClampPosition = 0.2;
        rightClampPosition = 0.2;
        rrClampPosition = 0;
        lrClampPosition = 0;
        liftPosistion = 0;
        extenderPosition = 0;

    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop()
    {

      /*
       * Gamepad 1
       *
       * Gamepad 1 controls the motors via the left stick, and it controls the
       * wrist/claw via the a,b, x, y buttons
       */

        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.
        float left = -gamepad1.left_stick_y;
        float right = -gamepad1.right_stick_y;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);

        // write the values to the motors
        motorBackRight.setPower(right);
        motorFrontRight.setPower(right);
        motorFrontLeft.setPower(left);
        motorBackLeft.setPower(left);


        // wrtie the value to the lift motor
        float up = -gamepad2.right_stick_y;
        up = (float)scaleInput(up);
        lift.setPower(up);

        float out = -gamepad2.left_stick_y;
        out = (float)scaleInput(out);
        extender.setPower(out);

        // update the position of the arm.
        if (gamepad1.b)
        {
            // if the B button is pushed on gamepad1, increase (by increment) the position of
            // the arm servo.
            rrClampPosition += clampDelta;
        }
        if (gamepad1.x)
        {
            // if the X button is pushed on gamepad1, decrease (by incrememnt) the position of
            // the arm servo.
            rrClampPosition -= clampDelta;
        }

        // update the position of the clamps
        if (gamepad1.dpad_left)
        {
            leftClampPosition -= clampDelta;
        }

        if (gamepad1.dpad_right)
        {
            leftClampPosition += clampDelta;
        }

        //test displacer
        /**if (gamepad1.dpad_up)
         {
         liftPosistion += liftDelta;
         }

         if (gamepad1.dpad_down)
         {
         liftPosistion -= liftDelta;
         }**/

        if (gamepad2.left_trigger != 0)
        {
            shoulderServo.setPower(0.00);
        }
        else if(gamepad2.left_bumper)
        {
            shoulderServo.setPower(1.00);
        }
        else
        {
            shoulderServo.setPower(0.50);
        }

        if (gamepad2.right_trigger != 0)
        {
            elbowServo.setPower(0.00);
        }
        else if(gamepad2.right_bumper)
        {
            elbowServo.setPower(1.00);
        }
        else
        {
            elbowServo.setPower(0.50);
        }

        if (gamepad2.b)
        {
            rrClampPosition += clampDelta;
        }
        if (gamepad2.x)
        {
            rrClampPosition -= clampDelta;
        }
        if (gamepad2.dpad_left)
        {
            lrClampPosition += clampDelta;
        }
        if (gamepad2.dpad_right)
        {
            lrClampPosition -= clampDelta;
        }



        // clip the position values so that they never exceed their allowed range.
        leftClampPosition = Range.clip(leftClampPosition, CLAMP_MIN_RANGE, CLAMP_MAX_RANGE);
        rightClampPosition = Range.clip(rightClampPosition, CLAMP_MIN_RANGE, CLAMP_MAX_RANGE);
        liftPosistion = Range.clip(liftPosistion, -0.7, 0.7);
        rrClampPosition = Range.clip(rrClampPosition, CLAMP_MIN_RANGE, CLAMP_MAX_RANGE);
        lrClampPosition = Range.clip(lrClampPosition, CLAMP_MIN_RANGE, CLAMP_MAX_RANGE);


        // write position values to the clamps
        leftClamp.setPosition(leftClampPosition);
        rightClamp.setPosition(rightClampPosition);
        lift.setPower(liftPosistion);
        extender.setPower(extenderPosition);
        arm1.setPosition(rrClampPosition);
        arm2.setPosition(lrClampPosition);
        //Close arms to clamp relic

      /*
       * Send telemetry data back to driver station. Note that if we are using
       * a legacy NXT-compatible motor controller, then the getPower() method
       * will return a null value. The legacy NXT-compatible motor controllers
       * are currently write only.
       */

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("leftClamp", "leftClamp:  " + String.format("%.2f", leftClampPosition));
        telemetry.addData("rightClamp", "rightClamp:  " + String.format("%.2f", rightClampPosition));
        telemetry.addData("left tgt pwr",  "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop()
    {

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double dScale;
    double scaleInput(double dVal)
    {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        }
        else if (index > 16)
        {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        }
        else
        {
            dScale = scaleArray[index];
        }

        return dScale;
    }

}