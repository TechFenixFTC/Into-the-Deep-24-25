package org.firstinspires.ftc.teamcode.robot.subsistemas.common.Horizontal;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.subsistemas.SubsistemasInferiores.Horizontal.LinearHorizontalStates;

import java.util.HashMap;

@Deprecated
public class LinearHorizontalV4 {
    public Servo servoLinearHorizontal;
    public static boolean monitor = false;
    private double targetPosition = 0;
    public double servoLinearHorizontalPosition;
    public HashMap<LinearHorizontalStates, Double> mapStateHorizontal = new HashMap<>();
    public LinearHorizontalStates linearHorizontalV4States = LinearHorizontalStates.RETRACTED;
    public LinearHorizontalV4(HardwareMap hardwareMap, String hardwareName) {
        this.servoLinearHorizontal = hardwareMap.get(Servo.class, hardwareName);

    }
    public Action goToExtended(){


        return new InstantAction(() -> {
            linearHorizontalV4States = LinearHorizontalStates.EXTENDED;
            servoLinearHorizontalPosition = mapStateHorizontal.get(linearHorizontalV4States);

        });
    }
    public Action goToRetracted(){


        return new InstantAction(() -> {
            linearHorizontalV4States = LinearHorizontalStates.RETRACTED;
            servoLinearHorizontalPosition = mapStateHorizontal.get(linearHorizontalV4States);


        });
    }
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