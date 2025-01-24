package org.firstinspires.ftc.teamcode.subsystems.common.Horizontal;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.HardwareNames;

@Config
public class LinearHorizontalV4 {
    public Servo servoLinearHorizontal;
    public static boolean monitor = false;
    public double targetPosition = 0;
    public LinearHorizontalV4(HardwareMap hardwareMap, String hardwareName) {
        this.servoLinearHorizontal = hardwareMap.get(Servo.class, hardwareName);

    }

    /**************************************************
     *                  Controllers                   *
     **************************************************/

    /**************************************************
     *                   Actions                      *
     **************************************************/
    /*REFAZER A FUNÇÕES PARA FUNCIONAREM COM SERVOS(SÃO FUNÇÕES BASEADAS NOS MOTORES)*/

    /**************************************************
     *              Controllers Tools                 *
     **************************************************/
    public void setTarget(double target) {
        targetPosition =  Range.clip(target, 0, 1);

    }


    /**************************************************
     *                   Monitoring                   *
     **************************************************/

    public void monitor(Telemetry telemetry,String hozizontal) {
        if (monitor) {
            telemetry.addLine("*********************************");
            telemetry.addData("TELEMETRIA DO LINEAR HORIZONTAL ",hozizontal);
            telemetry.addLine("*********************************");
            telemetry.addData("-Posição do Servo: ",servoLinearHorizontal.getPosition());
            telemetry.addData("-Posição alvo: ",targetPosition);
            telemetry.addData("-Porta: ", servoLinearHorizontal.getPortNumber());
        }
    }
}