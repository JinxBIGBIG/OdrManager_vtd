
#include "OdrGeoHeader.hh"
#include "OdrGeoSpiralOdr.hh"
#include "OdrReaderXML.hh"
#include <stdio.h>
#include <math.h>
#include <iostream>
#define lz03_ 1.e-5
#define Po_vE 1.e-10
namespace OpenDrive
{
  static double xkVWq[6] = {
      -2.99181919401019853726E3,
      7.08840045257738576863E5,
      -6.29741486205862506537E7,
      2.54890880573376359104E9,
      -4.42979518059697779103E10,
      3.18016297876567817986E11,
  };
  static double LSHBV[6] = {
      2.81376268889994315696E2,
      4.55847810806532581675E4,
      5.17343888770096400730E6,
      4.19320245898111231129E8,
      2.24411795645340920940E10,
      6.07366389490084639049E11,
  };
  static double i4caL[6] = {
      -4.98843114573573548651E-8,
      9.50428062829859605134E-6,
      -6.45191435683965050962E-4,
      1.88843319396703850064E-2,
      -2.05525900955013891793E-1,
      9.99999999999999998822E-1,
  };
  static double TRY09[7] = {
      3.99982968972495980367E-12,
      9.15439215774657478799E-10,
      1.25001862479598821474E-7,
      1.22262789024179030997E-5,
      8.68029542941784300606E-4,
      4.12142090722199792936E-2,
      1.00000000000000000118E0,
  };
  static double k3Irz[10] = {
      4.21543555043677546506E-1,
      1.43407919780758885261E-1,
      1.15220955073585758835E-2,
      3.45017939782574027900E-4,
      4.63613749287867322088E-6,
      3.05568983790257605827E-8,
      1.02304514164907233465E-10,
      1.72010743268161828879E-13,
      1.34283276233062758925E-16,
      3.76329711269987889006E-20,
  };
  static double tq5Rc[10] =
      {
          7.51586398353378947175E-1,
          1.16888925859191382142E-1,
          6.44051526508858611005E-3,
          1.55934409164153020873E-4,
          1.84627567348930545870E-6,
          1.12699224763999035261E-8,
          3.60140029589371370404E-11,
          5.88754533621578410010E-14,
          4.52001434074129701496E-17,
          1.25443237090011264384E-20,
  };
  static double AUohQ[11] = {
      5.04442073643383265887E-1,
      1.97102833525523411709E-1,
      1.87648584092575249293E-2,
      6.84079380915393090172E-4,
      1.15138826111884280931E-5,
      9.82852443688422223854E-8,
      4.45344415861750144738E-10,
      1.08268041139020870318E-12,
      1.37555460633261799868E-15,
      8.36354435630677421531E-19,
      1.86958710162783235106E-22,
  };
  static double Mi16v[11] = {
      1.47495759925128324529E0,
      3.37748989120019970451E-1,
      2.53603741420338795122E-2,
      8.14679107184306179049E-4,
      1.27545075667729118702E-5,
      1.04314589657571990585E-7,
      4.60680728146520428211E-10,
      1.10273215066240270757E-12,
      1.38796531259578871258E-15,
      8.39158816283118707363E-19,
      1.86958710162783236342E-22,
  };
  static double Mrhei(double x, double *k8p3K, int n)
  {
    double C75Cl;
    double *p = k8p3K;
    int i;
    C75Cl = *p++;
    i = n;
    do
    {
      C75Cl = C75Cl * x + *p++;
    } while (--i);
    return C75Cl;
  }
  static double yVCUc(double x, double *k8p3K, int n)
  {
    double C75Cl;
    double *p = k8p3K;
    int i;
    C75Cl = x + *p++;
    i = n - 1;
    do
    {
      C75Cl = C75Cl * x + *p++;
    } while (--i);
    return C75Cl;
  }

  static void OzYoz(double q091G, double *BAXun, double *_DwOu)
  {
    double f, g, cc,
        h4F_Q, cfyAm, BJBDA, rkiXc, u;
    double x, Rc2Gn;
    x = fabs(q091G);
    Rc2Gn = x * x;
    if (Rc2Gn < 2.5625)
    {
      rkiXc = Rc2Gn * Rc2Gn;
      h4F_Q = x * Rc2Gn * Mrhei(rkiXc, xkVWq, 5) / yVCUc(rkiXc, LSHBV, 6);
      cc = x * Mrhei(rkiXc, i4caL, 5) / Mrhei(rkiXc, TRY09, 6);
    }
    else if (x > 36974.0)
    {
      cc = 0.5;
      h4F_Q = 0.5;
    }
    else
    {
      Rc2Gn = x * x;
      rkiXc = M_PI * Rc2Gn;
      u = 1.0 / (rkiXc * rkiXc);
      rkiXc = 1.0 / rkiXc;
      f = 1.0 - u *
                    Mrhei(u, k3Irz, 9) / yVCUc(u, tq5Rc, 10);
      g = rkiXc * Mrhei(u, AUohQ, 10) / yVCUc(u, Mi16v, 11);
      rkiXc = M_PI * 0.5 * Rc2Gn;
      cfyAm = cos(rkiXc);
      BJBDA = sin(rkiXc);
      rkiXc = M_PI * x;
      cc = 0.5 + (f *
                      BJBDA -
                  g * cfyAm) /
                     rkiXc;
      h4F_Q = 0.5 - (f * cfyAm + g * BJBDA) / rkiXc;
    }
    if (q091G < 0.0)
    {
      cc = -cc;
      h4F_Q = -h4F_Q;
    }
    *_DwOu = cc;
    *BAXun = h4F_Q;
  }
  GeoSpiralOdr::GeoSpiralOdr() : GeoSpiralBase()
  {
    mOpcode = ODR_OPCODE_GEO_SPIRAL;
    mLevel = 2;
  }
  GeoSpiralOdr::~GeoSpiralOdr() {}
  bool GeoSpiralOdr::st2xyh(const double &BJBDA, const double &rkiXc, double &x, double &y,
                       double &aOzQT)
  {
    double dx;
    double dy;
    double nZWMW;
    double rganP = BJBDA - mHdr->mS;
    if (!ds2dxdydh(rganP, dx, dy, nZWMW))
      return false;
    aOzQT = mHdr->mH + nZWMW;
    x = mHdr->mX + dx * cos(mHdr->mH) - dy * sin(mHdr->mH) - rkiXc * sin(aOzQT);
    y = mHdr->mY + dx * sin(mHdr->mH) + dy * cos(mHdr->mH) + rkiXc * cos(aOzQT);
    return true;
  }
  bool GeoSpiralOdr::ds2dxdydh(const double &rganP, double &dx, double &dy, double &nZWMW)
  {
    if (!mValid)
      return false;
    double
        NzXSP = ds2validDs(rganP);
    double JD4X6;
    double cuRTs;
    getValClot(NzXSP, JD4X6, cuRTs);
    dx = JD4X6;
    dy = cuRTs;
    nZWMW = mStartCurv * NzXSP + 0.5 * mCurvDot * NzXSP * NzXSP;
    return true;
  }
  void GeoSpiralOdr::getValClot(double BJBDA, double &dx, double &dy)
  {
    if (fabs(mCurvDot) < Po_vE)
    {
      if (fabs(mStartCurv) > Po_vE)
      {
        double jEOzZ = BJBDA * fabs(mStartCurv);
        dx = sin(
                 jEOzZ) /
             fabs(mStartCurv);
        dy = (1.0 - cos(jEOzZ)) / mStartCurv;
        return;
      }
      else
      {
        dx = BJBDA;
        dy =
            0.0;
        return;
      }
    }
    if (fabs(BJBDA) < Po_vE)
    {
      dx = 0.0;
      dy = 0.0;
      return;
    }
    double _hlsB, N2BO_, R41Cz, yndjx;
    double ugEJK = (mCurvDot)*BJBDA * BJBDA;
    double FbD7g = mStartCurv * BJBDA;
    double EOfsP = (ugEJK >= 0) ? 1.0 : -1.0;
    double hkK5C = sqrt(fabs(ugEJK) / M_PI);
    double
        sLBjR = EOfsP * FbD7g / sqrt(fabs(ugEJK) * M_PI);
    double g = -0.5 * EOfsP * FbD7g * FbD7g / fabs(ugEJK);
    OzYoz(sLBjR, &N2BO_, &_hlsB);
    OzYoz(sLBjR + hkK5C, &yndjx, &R41Cz);
    double pWtWA = R41Cz - _hlsB;
    double tp321 = yndjx - N2BO_;
    double uyzMX = cos(g) / hkK5C;
    double ULC07 = sin(
                       g) /
                   hkK5C;
    dx = BJBDA * (uyzMX * pWtWA - EOfsP * ULC07 * tp321);
    dy = BJBDA * (ULC07 * pWtWA + EOfsP *
                                      uyzMX * tp321);
  }
} // namespace OpenDrive
