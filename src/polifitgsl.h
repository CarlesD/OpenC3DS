#ifndef _POLIFITGSL_H
#define _POLIFITGSL_H

#include <gsl/gsl_multifit.h>
#include <stdbool.h>
#include <math.h>

//COMENTARI Anna: polifitgsl ha de ser una classe?
bool polynomialfit(int obs, int degree, double *dx, double *dy, double *store); /* n, p */

#endif
