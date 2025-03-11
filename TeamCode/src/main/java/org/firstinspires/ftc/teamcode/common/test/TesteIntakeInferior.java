package org.firstinspires.ftc.teamcode.common.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.HardwareNames;

import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp(name = "TesteIntakeInferiorAn")
public class TesteIntakeInferior extends OpMode {

    List<Servo> servos = new ArrayList<>(4);
    String[] nomesServosTestados  = {"porta0c", "porta1c", "porta2c", "porta3c","porta4c"};

    double[] angulosServosTestado = {0, 0, 0, 0,0};
    int portaServoSendoTestado = 0;
    double cooldownChangePortaServo = 0;
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
        bracoGarraInferiorServo = hardwareMap.get(Servo.class, HardwareNames.bracoGarraInferiorServo);
        rotacaoGarraInferiorServo = hardwareMap.get(Servo.class, HardwareNames.rotacaoGarraInferiorServo);
        angulacaoGarraInferiorServo = hardwareMap.get(Servo.class, HardwareNames.rotacaoGarraInferiorServo);
        aberturaGarraInferiorServo = hardwareMap.get(Servo.class, HardwareNames.aberturaGarraInferiorServo);
        horizontalInferiorServo = hardwareMap.get(Servo.class, HardwareNames.horizontalInferiorServo);
        for (int i = 0; i < 5; i++) {
            servos.add(hardwareMap.get(Servo.class, nomesServosTestados[i]));
        }
    }

    @Override
    public void loop(){

        if(gamepad2.a || continous == true){

            horizontalInferiorServo.setPosition(0);
            aberturaGarraInferiorServo.setPosition(0.077);
            angulacaoGarraInferiorServo.setPosition(0.466);
            bracoGarraInferiorServo.setPosition(0.309);
            rotacaoGarraInferiorServo.setPosition(0.316);
        }
        if (gamepad1.dpad_right) {
            if (angulosServosTestado[portaServoSendoTestado] < 10.0) {  // Limite máximo
                angulosServosTestado[portaServoSendoTestado] += 0.001;
                servos.get(portaServoSendoTestado).setPosition(angulosServosTestado[portaServoSendoTestado]);
            }
        }

// Diminui o ângulo, com limite
        if (gamepad1.dpad_left) {
            if (angulosServosTestado[portaServoSendoTestado] > 0.0) {  // Limite mínimo
                angulosServosTestado[portaServoSendoTestado] -= 0.001;
                servos.get(portaServoSendoTestado).setPosition(angulosServosTestado[portaServoSendoTestado]);
            }
        }

        /* Muda a porta que está sendo alterada do servo */
        if(gamepad1.start) {
            if ( getRuntime() >=  cooldownChangePortaServo) {
                cooldownChangePortaServo = getRuntime() + 0.3;
                if (portaServoSendoTestado < 4) {
                    portaServoSendoTestado++;
                }
                else {
                    portaServoSendoTestado = 0;
                }
            }





        }
        // todo: write your code here
        telemetry.addLine("==== TESTES DE SERVO ====");
        telemetry.addData("Porta do servo", nomesServosTestados[portaServoSendoTestado]);
        telemetry.addData("Angulo do servo", angulosServosTestado[portaServoSendoTestado]);
        telemetry.addLine("============================");
        telemetry.update();

    }
    // todo: write your code here
}
