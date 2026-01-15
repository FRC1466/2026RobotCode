// Copyright (c) 2025-2026 Webb Robotics
// http://github.com/FRC1466

package frc.robot.util;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import java.util.List;
import java.util.Map;

public class ShooterModel {
  private static double[] distances;
  private static double[] velocities;
  private static double[] tofs;
  private static double[] aV, bV, cV, dV; // spline coeffs for velocity
  private static double[] aT, bT, cT, dT; // spline coeffs for time-of-flight

  public static void initialize(String jsonPath) {
    try {
      ObjectMapper mapper = new ObjectMapper();
      List<Map<String, Double>> data =
          mapper.readValue(new File(jsonPath), new TypeReference<List<Map<String, Double>>>() {});

      int n = data.size();
      distances = new double[n];
      velocities = new double[n];
      tofs = new double[n];

      for (int i = 0; i < n; i++) {
        Map<String, Double> p = data.get(i);
        distances[i] = p.get("distance");
        velocities[i] = p.get("velocity");
        tofs[i] = p.get("tof");
      }

      computeSplineCoefficients(distances, velocities);
      aV = splineA;
      bV = splineB;
      cV = splineC;
      dV = splineD;

      computeSplineCoefficients(distances, tofs);
      aT = splineA;
      bT = splineB;
      cT = splineC;
      dT = splineD;

    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  private static double[] splineA, splineB, splineC, splineD;

  private static void computeSplineCoefficients(double[] x, double[] y) {
    int n = x.length - 1;

    splineA = new double[n];
    splineB = new double[n];
    splineC = new double[n + 1];
    splineD = new double[n];

    double[] h = new double[n];
    for (int i = 0; i < n; i++) h[i] = x[i + 1] - x[i];

    double[] alpha = new double[n];
    for (int i = 1; i < n; i++)
      alpha[i] = (3.0 / h[i]) * (y[i + 1] - y[i]) - (3.0 / h[i - 1]) * (y[i] - y[i - 1]);

    double[] l = new double[n + 1];
    double[] mu = new double[n + 1];
    double[] z = new double[n + 1];
    l[0] = 1;
    mu[0] = 0;
    z[0] = 0;

    for (int i = 1; i < n; i++) {
      l[i] = 2 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
      mu[i] = h[i] / l[i];
      z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n] = 1;
    z[n] = 0;
    splineC[n] = 0;

    for (int j = n - 1; j >= 0; j--) {
      splineC[j] = z[j] - mu[j] * splineC[j + 1];
      splineB[j] = (y[j + 1] - y[j]) / h[j] - h[j] * (splineC[j + 1] + 2 * splineC[j]) / 3;
      splineD[j] = (splineC[j + 1] - splineC[j]) / (3 * h[j]);
      splineA[j] = y[j];
    }
  }

  private static double evalSpline(
      double[] x, double[] a, double[] b, double[] c, double[] d, double value) {
    int n = x.length - 1;
    int i = 0;
    if (value <= x[0]) i = 0;
    else if (value >= x[n]) i = n - 1;
    else {
      for (int j = 0; j < n; j++) {
        if (value >= x[j] && value <= x[j + 1]) {
          i = j;
          break;
        }
      }
    }
    double dx = value - x[i];
    return a[i] + b[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;
  }

  public static double getVelocity(double distance) {
    return evalSpline(distances, aV, bV, cV, dV, distance);
  }

  public static double getTimeOfFlight(double distance) {
    return evalSpline(distances, aT, bT, cT, dT, distance);
  }
}
