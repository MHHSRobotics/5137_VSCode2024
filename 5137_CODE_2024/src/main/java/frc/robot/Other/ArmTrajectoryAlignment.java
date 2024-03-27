package frc.robot.Other;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import org.apache.commons.math3.analysis.polynomials.PolynomialFunction;
import org.apache.commons.math3.analysis.solvers.LaguerreSolver;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

public class ArmTrajectoryAlignment {
    private LaguerreSolver solver;

    private double A_2;
    private double A_1;
    private double A_0;
    private double B_2;
    private double B_1;
    private double B_0;
    private double C_2;
    private double C_0;
    private double D_3;
    private double D_2;
    private double D_1;
    private double D_0;
    private double E_2;
    private double E_1;
    private double E_0;

    private double xo;
    private double xo2;
    private double xo3;

    private double minRange;
    private double maxRange;
    private double defaultAngle;

    private JSONParser parser;

    public ArmTrajectoryAlignment(File RobotConstants, double minRange, double maxRange, double defaultAngle) {
        this.minRange = minRange;
        this.maxRange = maxRange;
        this.defaultAngle = Math.toRadians(defaultAngle);

        solver = new LaguerreSolver();

        parser = new JSONParser();

        try {
            JSONObject obj = (JSONObject) parser.parse(new FileReader(RobotConstants));

            this.A_2 = (double) obj.get("A_2");
            this.A_1 = (double) obj.get("A_1");
            this.A_0 = (double) obj.get("A_0");
            this.B_2 = (double) obj.get("B_2");
            this.B_1 = (double) obj.get("B_1");
            this.B_0 = (double) obj.get("B_0");
            this.C_2 = (double) obj.get("C_2");
            this.C_0 = (double) obj.get("C_0");
            this.D_3 = (double) obj.get("D_3");
            this.D_2 = (double) obj.get("D_2");
            this.D_1 = (double) obj.get("D_1");
            this.D_0 = (double) obj.get("D_0");
            this.E_2 = (double) obj.get("E_2");
            this.E_1 = (double) obj.get("E_1");
            this.E_0 = (double) obj.get("E_0");

            this.xo = (double) obj.get("X Offset");
            this.xo2 = Math.pow(xo, 2);
            this.xo3 = Math.pow(xo, 3);

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (ParseException e) {
            e.printStackTrace();
        }
    }

    public double calculateAngle(double position) {
        if (position > minRange && position < maxRange) {
               
                double x = position;
                double x2 = Math.pow(x,2);
                double x3 = Math.pow(x,3);
                double x4 = Math.pow(x,4);

                return -Math.acos(solver.solve(
                100,
                new PolynomialFunction(
                    new double[]{
                        (x4 - (4*x3*xo) + (6*x2*xo2) - (4*x*xo3) + (E_2*x2) + (E_1*x*xo) + E_0),
                        ((D_3*(x3 - (3*x2*xo) + (3*x*xo2))) + (D_2*(2*x*xo - x2)) + (D_1*x) + D_0),
                        ((C_2*(x2 - 2*x*xo)) + C_0),
                        ((B_2*(x2 - 2*x*xo)) + (B_1*x) + B_0),
                        ((A_2*(x2 - 2*x*xo)) + (A_1*x) + A_0)
                    }
                ),
                0.5,
                1.0)) + (Math.PI/3);
            
        }
        else {
            return defaultAngle;
        }
    }
}
