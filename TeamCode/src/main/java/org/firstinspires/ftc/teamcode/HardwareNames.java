package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public final class HardwareNames {
    public static String

    /**************************************************
    *                  Rodas Mortas                   *
    **************************************************/
        par0 = "",
        par1 = "",
        perp = "",
    /**************************************************
     *                  Motores Chassi                *
     **************************************************/
        rightFront =  "rightFront",
        leftFront =   "leftFront",
        rightBack =   "rightBack",
        leftBack =    "leftBack",
    /**************************************************
    *                  Sensores                       *
    **************************************************/
        distanceSensorL      = "sensorporta3",
        distanceSensorR      = "sensorPorta2",
        colorSensor1         = "color1",
        colorSensor2         = "sensorPortaE2",

    /**************************************************
    *         Visão Computacional / Cameras          *
    **************************************************/
        webcam1              =  "Webcam 1",
        limelight            =  "limelight",
    /**************************************************
    *              DcMotors Subsistemas               *
    **************************************************/   /* Motores Vertex */
        verticalR            = "verticalr",
        verticalL            = "verticall",
        bracoGarraSuperior   = "b",
    /**************************************************
     *              Servo Subsistemas               *
     **************************************************/   /* Motores Vertex */

    horizontalSuperiorServo = "porta2c",
    horizontalInferiorServo = "porta4",
    bracoGarraSuperiorServo   = "porta3c",

    bracoGarraInferiorServo = "porta3",
    rotacaoGarraInferiorServo = "porta2",
    angulacaoGarraInferiorServo = "porta0",
    aberturaGarraInferiorServo = "porta1",

    /**************************************************
     *                 Servo-motores                  *
     **************************************************/
        servoRotacaoGarra    = "porta0c",
        servoAberturaGarrra  = "porta1c",
    /**************************************************
     *                 TroughBore                  *
     **************************************************/
    horizontalSuperior    = "braco",
    getHorizontalInferior  = "braco"
        ;

        public static String[] servoNames = {servoRotacaoGarra, servoAberturaGarrra};
}
