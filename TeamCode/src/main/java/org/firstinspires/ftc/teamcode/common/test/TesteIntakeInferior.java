package org.firstinspires.ftc.teamcode.common.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "TesteIntakeInferior")
public class TesteIntakeInferior extends OpMode {

    // portas
    // porta 0 -> horizontal inferior -> esticado: 1
    // porta 1 -> angulação da garra ->  transfer: 0.408
    // porta 2 -> braco garra -> intake -> 0.697 -> catch the sample: 0.712
    // porta 3 -> rotacao da garra
    // porta 4 -> abertura fechamento da garra

    Servo horizontalInferiorServo;
    Servo bracoGarraInferiorServo;
    Servo rotacaoGarraInferiorServo;
    Servo angulacaoGarraInferiorServo;
    Servo aberturaGarraInferiorServo;
    boolean continous = false;

    @Override
    public void init(){
        bracoGarraInferiorServo = hardwareMap.get(Servo.class, "bracoGarraInferiorServo");
        rotacaoGarraInferiorServo = hardwareMap.get(Servo.class, "rotacaoGarraInferiorServo");
        angulacaoGarraInferiorServo = hardwareMap.get(Servo.class, "angulacaoGarraInferiorServo");
        aberturaGarraInferiorServo = hardwareMap.get(Servo.class, "aberturaGarraInferiorServo");
        horizontalInferiorServo = hardwareMap.get(Servo.class, "horizontalInferiorServo");

    }

    @Override
    public void loop(){

        if(gamepad2.a || continous == true){

            horizontalInferiorServo.setPosition(1);
            aberturaGarraInferiorServo.setPosition(0.773);
            angulacaoGarraInferiorServo.setPosition(0.408);
            bracoGarraInferiorServo.setPosition(0.712);
            rotacaoGarraInferiorServo.setPosition(0.616);
        }

        if(gamepad2.x){
            horizontalInferiorServo.setPosition(1);
            aberturaGarraInferiorServo.setPosition(0.02);
            angulacaoGarraInferiorServo.setPosition(0.408);
            bracoGarraInferiorServo.setPosition(0.697);
            rotacaoGarraInferiorServo.setPosition(0.616);
        }
    }
    // todo: write your code here
}
