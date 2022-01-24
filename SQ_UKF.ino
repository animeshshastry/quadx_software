//Arduino/Teensy Flight Controller - quadx_firmware
//Author: Animesh Shastry
//========================================================================================================================//

#if (sq_UKF)

#include <Eigen.h>
#include <Core.h>
#include <Types.h>
#include <Integrator.h>
#include <StateVector.h>
#include <MeasurementVector.h>

//Point e3;
#if (!wind_est)
#define x_dim 8
#else
#define x_dim 11
#endif
#define y_dim 9

enum MyStateFields {
  Attitude,
  AngularVelocity,
#if (!wind_est)
  Velocity
#else
  Velocity,
  Wind
#endif
};

using MyStateVector = UKF::StateVector <
                      UKF::Field<Attitude, UKF::Vector<2>>,
                      UKF::Field<AngularVelocity, UKF::Vector<3>>,
#if (!wind_est)
                      UKF::Field<Velocity, UKF::Vector<3>>
#else
                      UKF::Field<Velocity, UKF::Vector<3>>,
                      UKF::Field<Wind, UKF::Vector<3>>
#endif
                      >;

namespace UKF {
namespace Parameters {
template <> constexpr real_t AlphaSquared<MyStateVector> = 1e-6;
//template <> constexpr real_t AlphaSquared<MyStateVector> = 25e-6;
}

template <> template <>
MyStateVector MyStateVector::derivative<>() const {
  MyStateVector temp;

  real_t s1 = get_field<Attitude>()[0];
  real_t s2 = get_field<Attitude>()[1];
  Point omega;
  omega(0) = get_field<AngularVelocity>()[0];
  omega(1) = get_field<AngularVelocity>()[1];
  omega(2) = get_field<AngularVelocity>()[2];
  Point v;
  v(0) = get_field<Velocity>()[0];
  v(1) = get_field<Velocity>()[1];
  v(2) = get_field<Velocity>()[2];
#if (wind_est)
  Point va;
  va(0) = get_field<Wind>()[0];
  va(1) = get_field<Wind>()[1];
  va(2) = get_field<Wind>()[2];
#endif

#if (!wind_est)
  Point Force_Aero = Aerodynamics(v);
#else
#if (wind_in_body_frame)
  Point Force_Aero = Aerodynamics(v-va);
#else
//  Point Force_Aero = Aerodynamics(v - (~R)*va);
  Point Force_Aero = Aerodynamics2(v+omega.CrossProduct(rCP),va);
#endif
#endif
  Point Torque_Aero = rCP.CrossProduct(Force_Aero) * 0.0;
  
  double s1_dot = 0.5 * (omega(1) * (1.0 + s1 * s1 - s2 * s2) + 2.0 * omega(2) * s2 - 2.0 * omega(0) * s1 * s2);
  double s2_dot = 0.5 * (omega(0) * (s1 * s1 - s2 * s2 - 1.0) - 2.0 * omega(2) * s1 + 2.0 * omega(1) * s1 * s2);

  ArrayMatrix<3, 1, double> RTe3 = { -2.0 * s1, -2.0 * s2, (1.0 - s1 * s1 - s2 * s2)};
  RTe3 = RTe3 * (1.0 / (1.0 + s1 * s1 + s2 * s2));

//  Serial.println(rc_knob);
  
  Point Iw = Inertia * omega;
  Point cross_term = omega.CrossProduct(Iw);
  Point omega_dot;
  if (rc_knob < 1.6) {
    omega_dot.Fill(0.0);
  }
  else{
    omega_dot = (Inertia.Inverse()) * (Torque_LP - cross_term + Torque_Aero);
  }
  
  /* Attitude derivative. */
  temp.set_field<Attitude>(UKF::Vector<2>(s1_dot, s2_dot));

  /* Angular velocity derivative. */
  temp.set_field<AngularVelocity>(UKF::Vector<3>(omega_dot(0), omega_dot(1), omega_dot(2)));

  /* Velocity derivative. */
  Point v_dot = (Force_Aero) / mass - omega.CrossProduct(v);
  if (rc_knob > 0.8) {
    v_dot += (e3 * Thrust_LP) / mass - RTe3 * gravity(2);
  }
  temp.set_field<Velocity>(UKF::Vector<3>(v_dot(0), v_dot(1), v_dot(2)));

#if (wind_est)
  /* Wind derivative. */
  Point va_dot;
#if (wind_in_body_frame)
  va_dot = -omega.CrossProduct(va);
#else
  va_dot.Fill(0.0);
#endif
  temp.set_field<Wind>(UKF::Vector<3>(va_dot(0), va_dot(1), va_dot(2)));
#endif

  return temp;
}

}

enum MyMeasurementFields {
  Accelerometer,
  Gyroscope,
  VIO_Velocity
};

using MyMeasurementVector = UKF::FixedMeasurementVector <
                            UKF::Field<Accelerometer, UKF::Vector<3>>,
                            UKF::Field<Gyroscope, UKF::Vector<3>>,
                            UKF::Field<VIO_Velocity, UKF::Vector<3>>
                            >;

using MyCore = UKF::SquareRootCore <
               MyStateVector,
               MyMeasurementVector,
               UKF::IntegratorRK4
               >;

namespace UKF {

template <> template <>
UKF::Vector<3> MyMeasurementVector::expected_measurement
<MyStateVector, Accelerometer>(const MyStateVector& state) {

  real_t s1 = state.get_field<Attitude>()[0];
  real_t s2 = state.get_field<Attitude>()[1];
  Point omega;
  omega(0) = state.get_field<AngularVelocity>()[0];
  omega(1) = state.get_field<AngularVelocity>()[1];
  omega(2) = state.get_field<AngularVelocity>()[2];
  Point v;
  v(0) = state.get_field<Velocity>()[0];
  v(1) = state.get_field<Velocity>()[1];
  v(2) = state.get_field<Velocity>()[2];
#if (wind_est)
  Point va;
  va(0) = state.get_field<Wind>()[0];
  va(1) = state.get_field<Wind>()[1];
  va(2) = state.get_field<Wind>()[2];
#endif

  ArrayMatrix<3, 1, double> RTe3 = { -2.0 * s1, -2.0 * s2, (1.0 - s1 * s1 - s2 * s2)};
  RTe3 = RTe3 * (1.0 / (1.0 + s1 * s1 + s2 * s2));
  
#if (!wind_est)
  Point Force_Aero = Aerodynamics(v);
#else
#if (wind_in_body_frame)
  Point Force_Aero = Aerodynamics(v-va);
#else
//  Point Force_Aero = Aerodynamics(v - (~R)*va);
  Point Force_Aero = Aerodynamics2(v+omega.CrossProduct(rCP),va);
#endif
#endif
  Point Torque_Aero = rCP.CrossProduct(Force_Aero) * 0.0;

  Point v_dot = (Force_Aero) / mass - omega.CrossProduct(v);
  if (rc_knob > 0.8) {
    v_dot += (e3 * Thrust_LP) / mass - RTe3 * gravity(2);
  }

  Point Iw = Inertia * omega;
  Point cross_term = omega.CrossProduct(Iw);
  Point omega_dot;
  if (rc_knob < 1.6) {
    omega_dot.Fill(0.0);
  }
  else{
    omega_dot = (Inertia.Inverse()) * (Torque_LP - cross_term + Torque_Aero);
  }
  
  Point wxrIMU = omega.CrossProduct(rIMU);
  Point alpha_IMU = omega.CrossProduct(wxrIMU) + omega_dot.CrossProduct(rIMU);

  ArrayMatrix<3, 1, double> accel_meas = v_dot + RTe3 * gravity(2) + alpha_IMU;

  return UKF::Vector<3>(accel_meas(0), accel_meas(1), accel_meas(2));
}

template <> template <>
UKF::Vector<3> MyMeasurementVector::expected_measurement
<MyStateVector, Gyroscope>(const MyStateVector& state) {
  return state.get_field<AngularVelocity>();
}

template <> template <>
UKF::Vector<3> MyMeasurementVector::expected_measurement
<MyStateVector, VIO_Velocity>(const MyStateVector& state) {
  return state.get_field<Velocity>();
}

}

static MyCore core;
static MyMeasurementVector meas;
void ukf_sensor_clear(void);
void ukf_sensor_set_accelerometer(real_t x, real_t y, real_t z);
void ukf_sensor_set_gyroscope(real_t x, real_t y, real_t z);
void ukf_sensor_set_VIO_Velocity(real_t x, real_t y, real_t z);

void ukf_init() {

  const double dt_ = 1.0 / loop_freq;

  /* Initialise state vector and covariance. */
  core.state.set_field<Attitude>(UKF::Vector<2>(0, 0));
  core.state.set_field<AngularVelocity>(UKF::Vector<3>(0, 0, 0));
  core.state.set_field<Velocity>(UKF::Vector<3>(0, 0, 0));
#if (wind_est)
  core.state.set_field<Wind>(UKF::Vector<3>(0, 0, 0));
#endif
  
  core.root_covariance = MyStateVector::CovarianceMatrix::Zero();
  core.root_covariance.diagonal() <<
     sqrt(1e-7) * UKF::Vector<2>::Ones() * sqrt(dt_) * sqrt(1e+3),
     sqrt(.3) * UKF::Vector<3>::Ones() * sqrt(dt_) * sqrt(1e+3),
#if (!wind_est)
     sqrt(.1) * UKF::Vector<3>::Ones() * sqrt(dt_) * sqrt(1e+3);
#else
     sqrt(.01) * UKF::Vector<3>::Ones() * sqrt(dt_) * sqrt(1e+3),
     sqrt(2e-7) * UKF::Vector<3>::Ones() * sqrt(dt_) * sqrt(1e+3);
#endif
     
  /* Set process noise covariance. */
  core.process_noise_root_covariance = MyStateVector::CovarianceMatrix::Zero();
  core.process_noise_root_covariance.diagonal() <<
     sqrt(1e-7) * UKF::Vector<2>::Ones() * sqrt(dt_),
     sqrt(.3) * UKF::Vector<3>::Ones() * sqrt(dt_),
#if (!wind_est)
     sqrt(.1) * UKF::Vector<3>::Ones() * sqrt(dt_);
#else
     sqrt(.01) * UKF::Vector<3>::Ones() * sqrt(dt_),
     sqrt(2e-7) * UKF::Vector<3>::Ones() * sqrt(dt_);
#endif
     
  /* Set measurement noise covariance. */
  core.measurement_root_covariance <<
     acc_noise * UKF::Vector<3>::Ones(),
     gyro_noise * UKF::Vector<3>::Ones(),
     VIO_noise * UKF::Vector<3>::Ones();
}

void ukf_iterate(float dt) {

  ukf_sensor_set_accelerometer(Acc(0), Acc(1), Acc(2));
  ukf_sensor_set_gyroscope(Gyro(0), Gyro(1), Gyro(2));
  ukf_sensor_set_VIO_Velocity(VIO_vel(0), VIO_vel(1), VIO_vel(2));

  core.a_priori_step(dt);
  core.innovation_step(meas);
  core.a_posteriori_step();

  StereoAtt(0) = core.state.get_field<Attitude>()[0];
  StereoAtt(1) = core.state.get_field<Attitude>()[1];
  omega(0) = core.state.get_field<AngularVelocity>()[0];
  omega(1) = core.state.get_field<AngularVelocity>()[1];
  omega(2) = core.state.get_field<AngularVelocity>()[2];
  body_vel(0) = core.state.get_field<Velocity>()[0];
  body_vel(1) = core.state.get_field<Velocity>()[1];
  body_vel(2) = core.state.get_field<Velocity>()[2];
#if (wind_est)
  wind_vel(0) = core.state.get_field<Wind>()[0];
  wind_vel(1) = core.state.get_field<Wind>()[1];
  wind_vel(2) = core.state.get_field<Wind>()[2];
#endif

  ArrayMatrix<y_dim, 1, double> y = { core.innovation.get_field<Accelerometer>()[0],
                                      core.innovation.get_field<Accelerometer>()[1],
                                      core.innovation.get_field<Accelerometer>()[2],
                                      core.innovation.get_field<Gyroscope>()[0],
                                      core.innovation.get_field<Gyroscope>()[1],
                                      core.innovation.get_field<Gyroscope>()[2],
                                      core.innovation.get_field<VIO_Velocity>()[0],
                                      core.innovation.get_field<VIO_Velocity>()[1],
                                      core.innovation.get_field<VIO_Velocity>()[2]};
//  // collision detection
//  if ((~y*y)(0, 0) > CRASH_THRES && current_time * micros2secs > 3 && !disarmed) {
//    crashed = true;
//  }

  // compute trace of the covariance matrix
  trace = 0.0;
  for (int i = 0; i < x_dim; i++) {
    trace += core.root_covariance(i, i)*core.root_covariance(i, i);
  }
//  Serial << "Trace: " << 1e+6*trace << '\n';
  
}

void ukf_sensor_clear() {
  meas = MyMeasurementVector();
}

void ukf_sensor_set_accelerometer(real_t x, real_t y, real_t z) {
  meas.set_field<Accelerometer>(UKF::Vector<3>(x, y, z));
}

void ukf_sensor_set_gyroscope(real_t x, real_t y, real_t z) {
  meas.set_field<Gyroscope>(UKF::Vector<3>(x, y, z));
}

void ukf_sensor_set_VIO_Velocity(real_t x, real_t y, real_t z) {
    meas.set_field<VIO_Velocity>(UKF::Vector<3>(x, y, z));
}

#endif
