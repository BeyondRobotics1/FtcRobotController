package org.firstinspires.ftc.teamcode.biobuzz.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BioBuzz TeleOp RED SCORE", group = "A")
public class TeleOpRedScore extends TeleOpRed {
    @Override
    protected StartingSide fixedStartingSide() {
        return StartingSide.SCORE;
    }
}
