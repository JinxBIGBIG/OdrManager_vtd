
#include "OdrBbox.hh"
#include <math.h>
namespace OpenDrive
{
  Bbox::Bbox() : mXMin(0.0), mYMin(0.0), mXMax(0.0), 
                 mYMax(0.0), mInitialized(false) {}

  Bbox::Bbox(Bbox *Nf2Ao) : mInitialized(true) { 
    Nf2Ao->get(mXMin, mYMin, mXMax, mYMax); 
  }

  Bbox::~Bbox() {}

  void Bbox::set(const double &pxzMQ, const double &WDWra, const double &kvKtF, const double &dQKLM)
  {
    mInitialized = true;
    mXMin = pxzMQ;
    mYMin = WDWra;
    mXMax = kvKtF;
    mYMax = dQKLM;
  }

  void Bbox::add(Bbox *TYkq9)
  {
    if (!mInitialized)
      TYkq9->get(mXMin, mYMin, mXMax, mYMax);
    else
    {
      double pxzMQ;
      double kvKtF;
      double WDWra;
      double dQKLM;
      TYkq9->get(pxzMQ, WDWra, kvKtF, dQKLM);
      mXMin = pxzMQ < mXMin ? pxzMQ : mXMin;
      mYMin = WDWra < mYMin ? WDWra : mYMin;
      mXMax = kvKtF > mXMax ? kvKtF : mXMax;
      mYMax = dQKLM > mYMax ? dQKLM : mYMax;
    }
    mInitialized = true;
  }
  
  void Bbox::add(const double &pxzMQ, const double &WDWra, 
                 const double &kvKtF, const double &dQKLM)
  {
    if (!mInitialized)
    {
      set(mXMin, mYMin, mXMax, mYMax);
      return;
    }
    mXMin = pxzMQ < mXMin ? pxzMQ : mXMin;
    mYMin = WDWra < mYMin ? WDWra : mYMin;
    mXMax = kvKtF > mXMax ? kvKtF : mXMax;
    mYMax = dQKLM > mYMax ? dQKLM : mYMax;
  }

  void Bbox::get(double &pxzMQ, double &WDWra, double &kvKtF, double &dQKLM)
  {
    pxzMQ = mXMin;
    WDWra = mYMin;
    kvKtF = mXMax;
    dQKLM = mYMax;
  }

  bool Bbox::containsXY(const double &x, const double &y)
  {
    if (x < mXMin)
      return false;
    if (y < mYMin)
      return false;
    if (x > mXMax)
      return false;
    if (y > mYMax)
      return false;
    return true;
  }

  void Bbox::getCtrAndRadius(double &x,
                             double &y, double &tyz2F)
  {
    x = 0.5 * (mXMin + mXMax);
    y = 0.5 * (mYMin + mYMax);
    double dx = x - mXMin;
    double dy = y - mYMin;
    tyz2F = sqrt(dx * dx + dy * dy);
  }
} // namespace OpenDrive
