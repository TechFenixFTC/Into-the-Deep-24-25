package org.firstinspires.ftc.teamcode.common;

public class Globals {

    public static boolean IS_SCORING = false;
    public static boolean IS_INTAKE = false;
    public static boolean IS_STORED = false;
    public static boolean IS_ARMED = false;

    public static void startPontuar() {
        IS_SCORING = true;
        IS_INTAKE = false;
    }

    public static void pararPontuar(){
        IS_SCORING = false;
        IS_INTAKE = false;
    }

    public static void startIntaking() {
        IS_SCORING = false;
        IS_INTAKE = true;
    }

    public static void pararIntaking() {
        IS_SCORING = false;
        IS_INTAKE = false;
    }
}
