/*
 * vectormath.h
 *
 *  Created on: Mar 7, 2014
 *      Author: afilippow
 */

#ifndef VECTORMATH_H_
#define VECTORMATH_H_
#include <vector>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <iostream>
#include <fstream>
using namespace std;
float scalar_product(vector<float> x, vector<float> y);
vector<float> vector_difference(vector<float> x, vector<float> y);
vector<float> vector_sum(vector<float> x, vector<float> y);
vector<float> derivative_distance(vector<float> position, vector<float> obstaclePosition);
vector<float> derivative_angle(vector<float> a, vector<float> b);
vector<float> scalar_product(float a, vector<float> x);
vector<float> vector_difference(vector<float> x, vector<float> y, string str);
float vector_length(vector<float> x);
float cosine_angle(vector<float> a, vector<float> b);
#endif /* VECTORMATH_H_ */
