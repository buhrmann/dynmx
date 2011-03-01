/*
 *  MathUtils.h
 *  cinder_dynmx
 *
 *  Created by Thomas Buhrmann on 24/01/2011.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

const float PI = 3.1415926535897932384626433832795f;
const float PI_OVER_180 = 0.017453292519943295769236907684886f;
const float PI_OVER_180_REC = 57.295779513082325f;
const float PI_OVER_TWO = 1.570796326794897f;
const float PI_OVER_FOUR = 0.785398163397448f;

const float DEG_TO_RAD = PI / 180.0;
const float RAD_TO_DEG = 180.0 / PI;

static const float degreesToRadians(float deg) { return deg * DEG_TO_RAD; };

static const float radiansToDegrees(float rad) { return rad * RAD_TO_DEG; };