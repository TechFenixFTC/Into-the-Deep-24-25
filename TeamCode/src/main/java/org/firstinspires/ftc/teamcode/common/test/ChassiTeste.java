package org.firstinspires.ftc.teamcode.common.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.HardwareNames;

public class ChassiTeste extends OpMode {

    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;
    DcMotorEx rightFront;
    double drive;
    double turn;
    double strafe;
    double fLeftPow, fRightPow, bLeftPow, bRightPow;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, HardwareNames.leftFront);
        leftBack = hardwareMap.get(DcMotorEx.class, HardwareNames.leftBack);
        rightBack = hardwareMap.get(DcMotorEx.class, HardwareNames.rightBack);
        rightFront = hardwareMap.get(DcMotorEx.class, HardwareNames.leftFront);
        // rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        // leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

// Reverse the right side motors
// Reverse left motors if you are using NeveRests


        drive = gamepad1.left_stick_y * -1;
        turn = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        fLeftPow = Range.clip(drive + turn + strafe, -1, 1);
        bLeftPow = Range.clip(drive + turn - strafe, -1, 1);
        fRightPow = Range.clip(drive - turn - strafe, -1, 1);
        bRightPow = Range.clip(drive - turn + strafe, -1, 1);

        leftFront.setPower(fLeftPow);
        leftBack.setPower(bLeftPow);
        rightFront.setPower(fRightPow);
        rightBack.setPower(bRightPow);

        telemetry.addData("corrente", leftFront.getCurrent(CurrentUnit.AMPS));
    }
}
