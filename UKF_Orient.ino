//Arduino/Teensy Flight Controller - quadx_firmware
//Author: Animesh Shastry
//========================================================================================================================//

#if (!ROS_COMM && !sq_UKF)

#include "VehicleParameters.h"
#define alpha 5e-3
#define kappa 0.0
#define beta 2.0

#define x_dim 5
#define y_dim 6

const double lambda = alpha * alpha * (x_dim + kappa) - x_dim;

ArrayMatrix<x_dim, 1, double> x;
ArrayMatrix < x_dim, 2 * x_dim + 1, double > x_sigma_pts, x_sigma_pts_pr;
ArrayMatrix<y_dim, 1, double> y;
ArrayMatrix < y_dim, 2 * x_dim + 1, double > y_sigma_pts;
ArrayMatrix<x_dim, x_dim, double> Px, Px_sqrt, Dist_x, D;
ArrayMatrix<x_dim, y_dim, double> Pxy;
ArrayMatrix<y_dim, y_dim, double> Py, N;
ArrayMatrix<x_dim, 1, double> ProcessModel(Matrix<x_dim, 1> x);
ArrayMatrix<y_dim, 1, double> MeasurementModel(Matrix<x_dim, 1> x);

void UKFSetup() {
  D.Fill(0.0);
  N.Fill(0.0);
  Px.Fill(0.0);

  const double dt = 1.0 / loop_freq;

  D(0, 0) = 1e-7; //s1
  D(1, 1) = 1e-7; //s2
  D(2, 2) = .3; //w1
  D(3, 3) = .3; //w2
  D(4, 4) = .3; //w3
  D = D * dt;

  N(0, 0) = acc_noise; //accx
  N(1, 1) = acc_noise; //accy
  N(2, 2) = acc_noise; //accz
  N(3, 3) = gyro_noise; //gyrox
  N(4, 4) = gyro_noise; //gyroy
  N(5, 5) = gyro_noise; //gyroz
  N = N*N;

  x(0) = 0.0; //s1
  x(1) = 0.0; //s2
  x(2) = 0.0; //w1
  x(3) = 0.0; //w2
  x(4) = 0.0; //w3

  Px = D * 1e+3;
  //  Px(0, 0) = 1.0; //s1
  //  Px(1, 1) = 1.0; //s2
  //  Px(2, 2) = 1.0; //w1
  //  Px(3, 3) = 1.0; //w2
  //  Px(4, 4) = 1.0; //w3

  //  Serial << "Setup D: " << D << '\n';
  //  Serial << "Setup N: " << N << '\n';
  //  Serial << "Setup Px: " << Px*1000.0 << '\n';
}

void UKFPropagate() {
  const double gamma = sqrt(x_dim + lambda);
  Px_sqrt = CholeskyDec5(Px);
  //  Px_sqrt = MatrixSquareRoot(Px);
  Dist_x = Px_sqrt * gamma;
  GenerateStateSigmaPts();
  PropagateStateSigmaPts();
  StateMeanAndCovariance();

  Px += D;
  //  Serial << "Px: " << Px * 100 << '\n';
  //  Serial << "Px_sqrt: " << Px_sqrt * 100 << '\n';
  //      Serial.print("sqrt_ok?: "); Serial.println((1e+3)*Determinant(Px - Px_sqrt * (~Px_sqrt)));
  //  Serial << "Dist_x: " << Dist_x * 1000 << '\n';
  //      Serial << "x_sigma_pts: " << x_sigma_pts*100 << '\n';
  //    Serial << "x_sigma_pts_pr: " << x_sigma_pts_pr*100 << '\n';
  //      Serial << "x_mean: " << x*100 << '\n';
  //  Serial << "Px: " << Px << '\n';
}

void UKFUpdate() {
  GenerateMeasSigmaPts();
  MeasMeanAndCovariance();
  //  Serial << "y_sigma_pts: " << y_sigma_pts;
  //  Serial << "y_mean: " << y;

  Py += N;

  ArrayMatrix<x_dim, y_dim, double> K = Pxy * (Py.Inverse());
  ArrayMatrix<y_dim, 1, double> meas = Acc && Gyro;
  x += K * (meas - y);
  Px -= K * (Py * (~K));

  StereoAtt(0) = x(0);
  StereoAtt(1) = x(1);
  omega(0) = x(2);
  omega(1) = x(3);
  omega(2) = x(4);

  if (((~(meas - y)) * (meas - y))(0, 0) > CRASH_THRES && current_time * micros2secs > 3 && !disarmed) {
    crashed = true;
//    Serial << " DELTA_Y: " << ((~(meas - y)) * (meas - y))(0, 0);
    //    Serial.println(current_time);
  }
  
  // compute trace of the covariance matrix
  trace = 0.0;
  for (int i = 0; i < x_dim; i++) {
    trace += Px(i, i);
  }
  
}

ArrayMatrix<x_dim, 1, double> ProcessModel(Matrix<x_dim, 1> x) {

  double s1 = x(0);
  double s2 = x(1);
  Point omega;
  omega(0) = x(2);
  omega(1) = x(3);
  omega(2) = x(4);

  double s1_dot = 0.5 * (omega(1) * (1.0 + s1 * s1 - s2 * s2) + 2.0 * omega(2) * s2 - 2.0 * omega(0) * s1 * s2);
  double s2_dot = 0.5 * (omega(0) * (s1 * s1 - s2 * s2 - 1.0) - 2.0 * omega(2) * s1 + 2.0 * omega(1) * s1 * s2);

  Matrix<3, 1> RTe3 = { -2.0 * s1, -2.0 * s2, (1.0 - s1 * s1 - s2 * s2)};
  RTe3 = RTe3 * (1.0 / (1.0 + s1 * s1 + s2 * s2));

  Point Iw = Inertia * omega;
  Point cross_term = omega.CrossProduct(Iw);
  Point omega_dot = (Inertia.Inverse()) * (- cross_term);

  ArrayMatrix<x_dim, 1, double> x_dot = {s1_dot, s2_dot, omega_dot(0), omega_dot(1), omega_dot(2)};
  return x + x_dot * dt;
}

ArrayMatrix<y_dim, 1, double> MeasurementModel(Matrix<x_dim, 1> x) {

  double s1 = x(0);
  double s2 = x(1);
  Point omega;
  omega(0) = x(2);
  omega(1) = x(3);
  omega(2) = x(4);

  Matrix<3, 1> RTe3 = { -2.0 * s1, -2.0 * s2, (1.0 - s1 * s1 - s2 * s2)};
  RTe3 = RTe3 * (1.0 / (1.0 + s1 * s1 + s2 * s2));

  Point Iw = Inertia * omega;
  Point cross_term = omega.CrossProduct(Iw);
  Point omega_dot = (Inertia.Inverse()) * (- cross_term);

  Point wxrIMU = omega.CrossProduct(rIMU);
  Point alpha_IMU = omega.CrossProduct(wxrIMU) + omega_dot.CrossProduct(rIMU);

  Point accel_meas = RTe3 * gravity(2) + alpha_IMU;
  //    Serial << "RTe3: " << RTe3*100 << '\n';

  //    Point accel_meas = v_dot + RTe3 * gravity(2) + alpha_IMU + AccBias;
  Point gyro_meas = omega;
  //  Point gyro_meas = omega + GyroBias;

  return accel_meas && gyro_meas;
}

void GenerateMeasSigmaPts() {
  y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 0, 1 > ()) = MeasurementModel(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 0, 1 > ()));
  y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 1, 2 > ()) = MeasurementModel(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 1, 2 > ()));
  y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 2, 3 > ()) = MeasurementModel(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 2, 3 > ()));
  y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 3, 4 > ()) = MeasurementModel(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 3, 4 > ()));
  y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 4, 5 > ()) = MeasurementModel(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 4, 5 > ()));
  y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 5, 6 > ()) = MeasurementModel(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 5, 6 > ()));
  y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 1, x_dim + 2 > ()) = MeasurementModel(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 1, x_dim + 2 > ()));
  y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 2, x_dim + 3 > ()) = MeasurementModel(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 2, x_dim + 3 > ()));
  y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 3, x_dim + 4 > ()) = MeasurementModel(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 3, x_dim + 4 > ()));
  y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 4, x_dim + 5 > ()) = MeasurementModel(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 4, x_dim + 5 > ()));
  y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 5, x_dim + 6 > ()) = MeasurementModel(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 5, x_dim + 6 > ()));
}

void MeasMeanAndCovariance() {
  double weight = lambda / (x_dim + lambda);
  y = y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 0, 1 > ()) * weight;
  //  Serial << "num:" << (lambda) << '\n';
  //  Serial << "den:" << (x_dim + lambda) << '\n';
  //  Serial << "weight:" << weight << '\n';
  //  Serial << "y_sigma_pts: " << y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 0, 1 > ()) << '\n';
  //  Serial << "y: " << y << '\n';

  weight = 0.5 / (x_dim + lambda);
  y += y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 1, 2 > ()) * weight;
  y += y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 2, 3 > ()) * weight;
  y += y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 3, 4 > ()) * weight;
  y += y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 4, 5 > ()) * weight;
  y += y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 5, 6 > ()) * weight;
  y += y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 1, x_dim + 2 > ()) * weight;
  y += y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 2, x_dim + 3 > ()) * weight;
  y += y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 3, x_dim + 4 > ()) * weight;
  y += y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 4, x_dim + 5 > ()) * weight;
  y += y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 5, x_dim + 6 > ()) * weight;

  weight = (lambda / (x_dim + lambda)) + (1.0 - alpha * alpha + beta);
  Py = (y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 0, 1 > ()) - y) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 0, 1 > ()) - y)) * weight;

  weight = 0.5 / (x_dim + lambda);
  Py += (y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 1, 2 > ()) - y) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 1, 2 > ()) - y)) * weight;
  Py += (y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 2, 3 > ()) - y) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 2, 3 > ()) - y)) * weight;
  Py += (y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 3, 4 > ()) - y) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 3, 4 > ()) - y)) * weight;
  Py += (y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 4, 5 > ()) - y) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 4, 5 > ()) - y)) * weight;
  Py += (y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 5, 6 > ()) - y) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 5, 6 > ()) - y)) * weight;
  Py += (y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 1, x_dim + 2 > ()) - y) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 1, x_dim + 2 > ()) - y)) * weight;
  Py += (y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 2, x_dim + 3 > ()) - y) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 2, x_dim + 3 > ()) - y)) * weight;
  Py += (y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 3, x_dim + 4 > ()) - y) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 3, x_dim + 4 > ()) - y)) * weight;
  Py += (y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 4, x_dim + 5 > ()) - y) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 4, x_dim + 5 > ()) - y)) * weight;
  Py += (y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 5, x_dim + 6 > ()) - y) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 5, x_dim + 6 > ()) - y)) * weight;

  weight = (lambda / (x_dim + lambda)) + (1.0 - alpha * alpha + beta);
  Pxy = (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 0, 1 > ()) - x) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 0, 1 > ()) - y)) * weight;

  weight = 0.5 / (x_dim + lambda);
  Pxy += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 1, 2 > ()) - x) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 1, 2 > ()) - y)) * weight;
  Pxy += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 2, 3 > ()) - x) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 2, 3 > ()) - y)) * weight;
  Pxy += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 3, 4 > ()) - x) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 3, 4 > ()) - y)) * weight;
  Pxy += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 4, 5 > ()) - x) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 4, 5 > ()) - y)) * weight;
  Pxy += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 5, 6 > ()) - x) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < 5, 6 > ()) - y)) * weight;
  Pxy += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 1, x_dim + 2 > ()) - x) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 1, x_dim + 2 > ()) - y)) * weight;
  Pxy += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 2, x_dim + 3 > ()) - x) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 2, x_dim + 3 > ()) - y)) * weight;
  Pxy += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 3, x_dim + 4 > ()) - x) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 3, x_dim + 4 > ()) - y)) * weight;
  Pxy += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 4, x_dim + 5 > ()) - x) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 4, x_dim + 5 > ()) - y)) * weight;
  Pxy += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 5, x_dim + 6 > ()) - x) * (~(y_sigma_pts.Submatrix(Slice<0, y_dim>(), Slice < x_dim + 5, x_dim + 6 > ()) - y)) * weight;
}

void StateMeanAndCovariance() {
  double weight = lambda / (x_dim + lambda);
  x = x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 0, 1 > ()) * weight;

  weight = 0.5 / (x_dim + lambda);
  x += x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 1, 2 > ()) * weight;
  x += x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 2, 3 > ()) * weight;
  x += x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 3, 4 > ()) * weight;
  x += x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 4, 5 > ()) * weight;
  x += x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 5, 6 > ()) * weight;
  x += x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 1, x_dim + 2 > ()) * weight;
  x += x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 2, x_dim + 3 > ()) * weight;
  x += x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 3, x_dim + 4 > ()) * weight;
  x += x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 4, x_dim + 5 > ()) * weight;
  x += x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 5, x_dim + 6 > ()) * weight;

  weight = (lambda / (x_dim + lambda)) + (1.0 - alpha * alpha + beta);
  Px = (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 0, 1 > ()) - x) * (~(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 0, 1 > ()) - x)) * weight;

  weight = 0.5 / (x_dim + lambda);
  Px += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 1, 2 > ()) - x) * (~(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 1, 2 > ()) - x)) * weight;
  Px += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 2, 3 > ()) - x) * (~(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 2, 3 > ()) - x)) * weight;
  Px += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 3, 4 > ()) - x) * (~(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 3, 4 > ()) - x)) * weight;
  Px += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 4, 5 > ()) - x) * (~(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 4, 5 > ()) - x)) * weight;
  Px += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 5, 6 > ()) - x) * (~(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 5, 6 > ()) - x)) * weight;
  Px += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 1, x_dim + 2 > ()) - x) * (~(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 1, x_dim + 2 > ()) - x)) * weight;
  Px += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 2, x_dim + 3 > ()) - x) * (~(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 2, x_dim + 3 > ()) - x)) * weight;
  Px += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 3, x_dim + 4 > ()) - x) * (~(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 3, x_dim + 4 > ()) - x)) * weight;
  Px += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 4, x_dim + 5 > ()) - x) * (~(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 4, x_dim + 5 > ()) - x)) * weight;
  Px += (x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 5, x_dim + 6 > ()) - x) * (~(x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 5, x_dim + 6 > ()) - x)) * weight;
}

void PropagateStateSigmaPts() {
  x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 0, 1 > ()) = ProcessModel(x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < 0, 1 > ()));
  x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 1, 2 > ()) = ProcessModel(x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < 1, 2 > ()));
  x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 2, 3 > ()) = ProcessModel(x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < 2, 3 > ()));
  x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 3, 4 > ()) = ProcessModel(x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < 3, 4 > ()));
  x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 4, 5 > ()) = ProcessModel(x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < 4, 5 > ()));
  x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < 5, 6 > ()) = ProcessModel(x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < 5, 6 > ()));
  x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 1, x_dim + 2 > ()) = ProcessModel(x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 1, x_dim + 2 > ()));
  x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 2, x_dim + 3 > ()) = ProcessModel(x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 2, x_dim + 3 > ()));
  x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 3, x_dim + 4 > ()) = ProcessModel(x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 3, x_dim + 4 > ()));
  x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 4, x_dim + 5 > ()) = ProcessModel(x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 4, x_dim + 5 > ()));
  x_sigma_pts_pr.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 5, x_dim + 6 > ()) = ProcessModel(x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 5, x_dim + 6 > ()));
}

void GenerateStateSigmaPts() {
  x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < 0, 1 > ()) = x;
  x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < 1, 2 > ()) = x + Dist_x.Submatrix(Slice<0, x_dim>(), Slice < 0, 1 > ());
  x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < 2, 3 > ()) = x + Dist_x.Submatrix(Slice<0, x_dim>(), Slice < 1, 2 > ());
  x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < 3, 4 > ()) = x + Dist_x.Submatrix(Slice<0, x_dim>(), Slice < 2, 3 > ());
  x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < 4, 5 > ()) = x + Dist_x.Submatrix(Slice<0, x_dim>(), Slice < 3, 4 > ());
  x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < 5, 6 > ()) = x + Dist_x.Submatrix(Slice<0, x_dim>(), Slice < 4, 5 > ());
  x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 1, x_dim + 2 > ()) = x - Dist_x.Submatrix(Slice<0, x_dim>(), Slice < 0, 1 > ());
  x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 2, x_dim + 3 > ()) = x - Dist_x.Submatrix(Slice<0, x_dim>(), Slice < 1, 2 > ());
  x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 3, x_dim + 4 > ()) = x - Dist_x.Submatrix(Slice<0, x_dim>(), Slice < 2, 3 > ());
  x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 4, x_dim + 5 > ()) = x - Dist_x.Submatrix(Slice<0, x_dim>(), Slice < 3, 4 > ());
  x_sigma_pts.Submatrix(Slice<0, x_dim>(), Slice < x_dim + 5, x_dim + 6 > ()) = x - Dist_x.Submatrix(Slice<0, x_dim>(), Slice < 4, 5 > ());
}

void printUKFRollPitchYaw(int print_rate) {
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
    print_counter = micros();
    SERIAL_PORT.print(F(" UKF_roll: "));
    SERIAL_PORT.print(rpy_UKF(0)*rad2deg);
    SERIAL_PORT.print(F(" UKF_pitch: "));
    SERIAL_PORT.print(rpy_UKF(1)*rad2deg);
    SERIAL_PORT.print(F(" UKF_yaw: "));
    SERIAL_PORT.print(rpy_UKF(2)*rad2deg);
  }
}

void printUKFStereo(int print_rate) {
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
    print_counter = micros();
    SERIAL_PORT.print(F(" UKF_s1 : "));
    SERIAL_PORT.print(x(0));
    SERIAL_PORT.print(F(" UKF_s2: "));
    SERIAL_PORT.print(x(1));
  }
}

void printUKFOmega(int print_rate) {
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
    print_counter = micros();
    SERIAL_PORT.print(F(" UKF_wx : "));
    SERIAL_PORT.print(x(2)*rad2deg);
    SERIAL_PORT.print(F(" UKF_wy: "));
    SERIAL_PORT.print(x(3)*rad2deg);
    SERIAL_PORT.print(F(" UKF_wz: "));
    SERIAL_PORT.print(x(4)*rad2deg);
  }
}

void printUKFAccEst(int print_rate) {
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
    print_counter = micros();
    SERIAL_PORT.print(F(" UKF_accx : "));
    SERIAL_PORT.print(y(0));
    SERIAL_PORT.print(F(" UKF_accy: "));
    SERIAL_PORT.print(y(1));
    SERIAL_PORT.print(F(" UKF_accz: "));
    SERIAL_PORT.print(y(2));
  }
}

void printUKFGyroEst(int print_rate) {
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
    print_counter = micros();
    SERIAL_PORT.print(F(" UKF_gyrox : "));
    SERIAL_PORT.print(y(3)*rad2deg);
//    SERIAL_PORT.print(F(" UKF_gyroy: "));
//    SERIAL_PORT.print(y(4)*rad2deg);
//    SERIAL_PORT.print(F(" UKF_gyroz: "));
//    SERIAL_PORT.print(y(5)*rad2deg);
  }
}

void printTrace(int print_rate) {
  if ( (current_time - print_counter) * micros2secs > (1.0 / print_rate)) {
    print_counter = micros();
    SERIAL_PORT.print(F(" Px_trace: "));
//    double trace = 0.0;
//    for (int i = 0; i < x_dim; i++) {
//      trace += Px(i, i);
//    }
    SERIAL_PORT.print(trace);
  }
}

#endif
