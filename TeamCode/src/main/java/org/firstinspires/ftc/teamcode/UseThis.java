package org.firstinspires.ftc.teamcode;
//imports libraries
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
//name of code
@TeleOp (name="USE THIS")
public class UseThis extends LinearOpMode {
    //Setting Variables
    double tgtPower = 0;
    //Drive motors
    private DcMotor motorfrontLeft;
    private DcMotor motorfrontRight;
    private DcMotor motorbackLeft;
    private DcMotor motorbackRight;
    //Arm Motors
    private DcMotor LoMotorArm;
    private DcMotor HiMotorArm;
    //Servos
    private CRServo grabber;
    private CRServo servoarm;
    
    //CODE STARTS HERE
    @Override
    public void runOpMode() throws InterruptedException {
        //Initializes drive motors
        motorfrontLeft = hardwareMap.dcMotor.get("leftFront");
        motorfrontRight = hardwareMap.dcMotor.get("rightFront");
        motorbackLeft = hardwareMap.dcMotor.get("leftBack");
        motorbackRight = hardwareMap.dcMotor.get("rightBack");
        //Initializes arm motors
        LoMotorArm = hardwareMap.dcMotor.get("LoArm");
        HiMotorArm = hardwareMap.dcMotor.get("HiArm");
        //Initializes servos
        servoarm = hardwareMap.crservo.get("servoarm");
        grabber = hardwareMap.crservo.get("grabber");
        //Initializes motor directions
        motorfrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorfrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorbackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorbackRight.setDirection(DcMotor.Direction.FORWARD);
        
        //run portion
        waitForStart();
        while (opModeIsActive()) {
            //Move forward
            if (gamepad1.left_stick_y > 0) {
                motorfrontLeft.setPower(1);
                motorfrontRight.setPower(1);
                motorbackLeft.setPower(1);
                motorbackRight.setPower(1);
            }
            //Move back
            else if (gamepad1.left_stick_y < 0) {
                motorfrontLeft.setPower(-1);
                motorfrontRight.setPower(-1);
                motorbackLeft.setPower(-1);
                motorbackRight.setPower(-1);
            }
            //if y direction not moved power=0
            else {
                motorfrontLeft.setPower(0);
                motorfrontRight.setPower(0);
                motorbackLeft.setPower(0);
                motorbackRight.setPower(0);
            }
            //Strafe right
            if (gamepad1.dpad_right) {
                motorfrontLeft.setPower(1);
                motorfrontRight.setPower(-1);
                motorbackLeft.setPower(-1);
                motorbackRight.setPower(1);
            }
            //Strafe left
            else if (gamepad1.dpad_left) {
                motorfrontLeft.setPower(-1);
                motorfrontRight.setPower(1);
                motorbackLeft.setPower(1);
                motorbackRight.setPower(-1);
            } 
            //if dpad not touched power=0
            else {
                motorfrontLeft.setPower(0);
                motorfrontRight.setPower(0);
                motorbackLeft.setPower(0);
                motorbackRight.setPower(0);
            }
            //turn right
            if (gamepad1.right_stick_x > 0) {
                motorfrontLeft.setPower(-1);
                motorbackLeft.setPower(-1);
                motorfrontRight.setPower(1);
                motorbackRight.setPower(1);
            } 
            //turn left
            else if (gamepad1.right_stick_x < 0) {
                motorfrontRight.setPower(-1);
                motorbackRight.setPower(-1);
                motorfrontLeft.setPower(1);
                motorbackLeft.setPower(1);
            }
            //if stick not touched in x direction power=0
            else {
                motorfrontLeft.setPower(0);
                motorfrontRight.setPower(0);
                motorbackLeft.setPower(0);
                motorbackRight.setPower(0);
            }
            //ALL CODE HERE MOVES @25% speed
            //Move forward
            if (gamepad1.left_stick_y > 0 && gamepad1.a) {
                motorfrontLeft.setPower(0.25);
                motorfrontRight.setPower(0.25);
                motorbackLeft.setPower(0.25);
                motorbackRight.setPower(0.25);
            }
            //Move back
            else if (gamepad1.left_stick_y < 0 && gamepad1.a) {
                motorfrontLeft.setPower(-0.25);
                motorfrontRight.setPower(-0.25);
                motorbackLeft.setPower(-0.25);
                motorbackRight.setPower(-0.25);
            }
            //if y direction not moved power=0
            else {
                motorfrontLeft.setPower(0);
                motorfrontRight.setPower(0);
                motorbackLeft.setPower(0);
                motorbackRight.setPower(0);
            }
            //Strafe right
            if (gamepad1.dpad_right && gamepad1.a) {
                motorfrontLeft.setPower(0.25);
                motorfrontRight.setPower(-0.25);
                motorbackLeft.setPower(-0.25);
                motorbackRight.setPower(0.25);
            }
            //Strafe left
            else if (gamepad1.dpad_left && gamepad1.a) {
                motorfrontLeft.setPower(-0.25);
                motorfrontRight.setPower(0.25);
                motorbackLeft.setPower(0.25);
                motorbackRight.setPower(-0.25);
            } 
            //if dpad not touched power=0
            else {
                motorfrontLeft.setPower(0);
                motorfrontRight.setPower(0);
                motorbackLeft.setPower(0);
                motorbackRight.setPower(0);
            }
            //turn right
            if (gamepad1.right_stick_x > 0 && gamepad1.a) {
                motorfrontLeft.setPower(-0.25);
                motorbackLeft.setPower(-0.25);
                motorfrontRight.setPower(0.25);
                motorbackRight.setPower(0.25);
            } 
            //turn left
            else if (gamepad1.right_stick_x < 0 && gamepad1.a) {
                motorfrontRight.setPower(-0.25);
                motorbackRight.setPower(-0.25);
                motorfrontLeft.setPower(0.25);
                motorbackLeft.setPower(0.25);
            }
            //if stick not touched in x direction power=0
            else {
                motorfrontLeft.setPower(0);
                motorfrontRight.setPower(0);
                motorbackLeft.setPower(0);
                motorbackRight.setPower(0);
            }
            //END OF 50% SPEED
            // Arm up
            if (gamepad2.dpad_up) {
                HiMotorArm.setPower(0.75);
            }
            // Arm Down
            else if (gamepad2.dpad_down) {
                HiMotorArm.setPower(-0.75);
            }
            //if dpad not touched power=0
            else {
                HiMotorArm.setPower(0);
            }
            //Arm Forwards
            if (gamepad2.dpad_right) {
                LoMotorArm.setPower(-1);
            }
            //Arm Backwards
            else if (gamepad2.dpad_left) {
                LoMotorArm.setPower(1);
            }
            //if dpad not touched power=0
            else {
                LoMotorArm.setPower(0);
            }
            //Plate grabber go down?
            if (gamepad2.x) {
                grabber.setPower(1);
            }
            //Plate Grabber go up?
            else if (gamepad2.y) {
                grabber.setPower(-1);
            }
            //if x or y not touched power=0
            else {
                grabber.setPower(0);
            }
            //Servo Arm open?
            if (gamepad2.a) {
                servoarm.setPower(1);
            }
            //Plate Grabber close?
            if (gamepad2.b) {
                servoarm.setPower(-1);
            }
        }
    }
}
