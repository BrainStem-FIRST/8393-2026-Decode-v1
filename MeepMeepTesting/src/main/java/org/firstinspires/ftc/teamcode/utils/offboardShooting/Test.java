package org.firstinspires.ftc.teamcode.utils.offboardShooting;

import java.util.function.Function;

public class Test {
    public static Function<Double, Double> getTpsFunction = mps -> Math.exp((mps + 32.33448) / 5.93989);

    public static void main(String[] args) {
        System.out.println(getTpsFunction.apply(0.17));
    }

}
