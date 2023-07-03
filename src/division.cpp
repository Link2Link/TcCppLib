#include "CHEN/division.h"

DivisionResult Division::divide() {
  if (fraction.denominator == 0L) throw DivisionByZero();

  DivisionResult result = DivisionResult{
      fraction.numerator / fraction.denominator,
      fraction.numerator % fraction.denominator
  };

  return result;
}


double calc(int type, double a, double b)
{
    if (type == 1)
    {
        return a*b;
    }
    else
    {
        return a+b;
    }
}