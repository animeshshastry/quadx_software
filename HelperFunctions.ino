//Arduino/Teensy Flight Controller - quadx_firmware
//Author: Animesh Shastry

//========================================================================================================================//

Point EulerAnglesFrom(Rotation R) {
  Matrix<3, 2> Eul_set = R.ToEulerAngles();
  //  Serial << "Euler angles: " << Eul_set  << "\n";
  Point rpy;
  rpy(0) = Eul_set(0, 0);
  rpy(1) = Eul_set(1, 0);
  rpy(2) = Eul_set(2, 0);
  return rpy;
}

Matrix<3, 1> vee(Matrix<3, 3> A) {
  Matrix<3, 1> vec = {A(2, 1), A(0, 2), A(1, 0)};
  return vec;
}

Matrix<3, 3> hat(Point a) {
  Matrix<3, 3> mat = {0.0, -a(2), a(1),
                      a(2), 0.0, -a(0),
                      -a(1), a(0), 0.0
                     };
  return mat;
}

Point Aerodynamics(Point v) {
  Point Force_Aero;
  Force_Aero(0) = -CD(0) * v(0);
  Force_Aero(1) = -CD(1) * v(1);
  Force_Aero(2) = -CD(2) * v(2);
  //  Force_Aero(0) = -CD(0) * abs(v(0)) * v(0);
  //  Force_Aero(1) = -CD(1) * abs(v(1)) * v(1);
  //  Force_Aero(2) = -CD(2) * abs(v(2)) * v(2);
  return Force_Aero;
}

ArrayMatrix<3, 1, double> CrossProduct(ArrayMatrix<3, 3, double> a, ArrayMatrix<3, 3, double> b) {
    ArrayMatrix<3,1,double> ret;

    ret(0) = a(1) * b(2) - a(2) * b(1);
    ret(1) = a(2) * b(0) - a(0) * b(2);
    ret(2) = a(0) * b(1) - a(1) * b(0);

    return ret;
}

ArrayMatrix<5, 5, double> CholeskyDec5(Matrix<5, 5> A) {
  //outputs lower diagonal matrix
#define precision_zero 1e-10
  unsigned int NumCol = A.GetColCount();
  unsigned int NumRow = A.GetRowCount();
  /* Note that outp need to be initialized as zero matrix */
  ArrayMatrix<5, 5, double> outp;
  outp.Fill(0.0);

  for (int16_t j = 0; j < NumCol; j++) {
    for (int16_t i = j; i < NumRow; i++) {
      double temp_d = A(i, j);
      if (i == j) {
        for (int16_t _k = 0; _k < j; _k++) {
          temp_d = temp_d - (outp(i, _k) * outp(i, _k));
        }
        if (temp_d < -precision_zero) {
          /* Matrix is not positif (semi)definit */
          temp_d = -temp_d;
        }
        /* Rounding to zero to avoid case where sqrt(0-) */
        if (abs(temp_d) < precision_zero) {
          temp_d = 0.0;
        }
        outp(i, i) = sqrt(temp_d);
      } else {
        for (int16_t _k = 0; _k < j; _k++) {
          temp_d = temp_d - (outp(i, _k) * outp(j, _k));
        }
        //        outp(i, j) = temp_d / outp(j, j);
        if (abs(outp(j, j)) < precision_zero) outp(i, j) = 0.0;  /* Matrix is not positif definit */
        else outp(i, j) = temp_d / outp(j, j);
      }
    }
  }
  return outp;
}

ArrayMatrix<8, 8, double> CholeskyDec8(Matrix<8, 8> A) {
  //outputs lower diagonal matrix
#define precision_zero 1e-10
  unsigned int NumCol = A.GetColCount();
  unsigned int NumRow = A.GetRowCount();
  /* Note that outp need to be initialized as zero matrix */
  ArrayMatrix<8, 8, double> outp;
  outp.Fill(0.0);

  for (int16_t j = 0; j < NumCol; j++) {
    for (int16_t i = j; i < NumRow; i++) {
      double temp_d = A(i, j);
      if (i == j) {
        for (int16_t _k = 0; _k < j; _k++) {
          temp_d = temp_d - (outp(i, _k) * outp(i, _k));
        }
        if (temp_d < -precision_zero) {
          /* Matrix is not positif (semi)definit */
          temp_d = -temp_d;
        }
        /* Rounding to zero to avoid case where sqrt(0-) */
        if (abs(temp_d) < precision_zero) {
          temp_d = 0.0;
        }
        outp(i, i) = sqrt(temp_d);
      } else {
        for (int16_t _k = 0; _k < j; _k++) {
          temp_d = temp_d - (outp(i, _k) * outp(j, _k));
        }
        //        outp(i, j) = temp_d / outp(j, j);
        if (abs(outp(j, j)) < precision_zero) outp(i, j) = 0.0;  /* Matrix is not positif definit */
        else outp(i, j) = temp_d / outp(j, j);
      }
    }
  }
  return outp;
}
