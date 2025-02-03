package org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Garra.GarraRotationInferiorStates;

import java.util.HashMap;

public class IntakeSuccao {
    public Servo angulacao;
    public DcMotor sugador;
    public Servo alcapao;
    public HashMap<GarraRotationInferiorStates, Double> mapRotation = new HashMap<>();

    public IntakeSuccao(HardwareMap hardwareMap){
        angulacao = hardwareMap.get(Servo.class,"");
        sugador = hardwareMap.get(DcMotor.class,"");
        alcapao = hardwareMap.get(Servo.class,"");

    }
    public Action intake(){
        return new InstantAction(()->{

        });
    }
}
