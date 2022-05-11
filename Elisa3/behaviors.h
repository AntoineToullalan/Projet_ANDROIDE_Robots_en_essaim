
#ifndef BEHAVIORS_H
#define BEHAVIORS_H

/**
 * \file behaviors.h
 * \brief Behaviors module
 * \author Stefano Morgani <stefano@gctronic.com>
 * \version 1.0
 * \date 01.02.12
 * \copyright GNU GPL v3

This module contains two basic low level behaviors: obstacle and cliff avoidance. Obstacle avoidance is
accomplished using the 8 proximity sensors placed around the robot, whereas the cliff avoidance is based
on the 4 ground sensors values.

*/


#include "variables.h"
#include <math.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief Initialize variables and functionalities (e.g. random generator) used in
 * the behaviors.
 * \return none
 */
void initBehaviors();

/**
 * \brief Obstacle avoidance behavior based on a simplified force field method.
 * It works for both forward and backward motion. The function need to be called before
 * the speed controller and with a cadency of at least 122 Hz (motors pwm frequency).
 * \return none
 */
void obstacleAvoidance(signed int *pwmLeft, signed int *pwmRight);

/**
 * \brief Cliff avoidance simple implementation in which the robot can be stopped when
 * a cliff is detected with the ground sensors. The robot has to be calibrated in the surface
 * in which it is moving in order for the behavior to work. The function need to be called before
 * the speed controller and with a cadency of at least 122 Hz (motors pwm frequency).
 * THE FUNCTION IS DEPRECATED: the behavior is directly inserted in the ADC interrupt service routine,
 * thus this function is no more needed and should not be called anymore.
 * \retval 0 cliff not detected
 * \retval 1 cliff detected
 */
char cliffDetected();

/**
 * \brief Enable obstacle avoidance behavior
 * \return none
 */
void enableObstacleAvoidance();

/**
 * \brief Disable obstacle avoidance behavior
 * \return none
 */
void disableObstacleAvoidance();

/**
 * \brief Enable cliff avoidance behavior
 * \return none
 */
void enableCliffAvoidance();

/**
 * \brief Disable cliff avoidance behavior
 * \return none
 */
void disableCliffAvoidance();



void aggregationBehavior(signed int *pwmLeft, signed int *pwmRight);

void enableAggregation();

void disableAggregation();


void followLeader(signed int *pwmLeft, signed int *pwmRight);

void enableFollowLeader();

void disableFollowLeader();


void avoidance(signed int *pwmLeft, signed int *pwmRight);

void enableAvoidance();

void disableAvoidance();


void wallFollow(signed int *pwmLeft, signed int *pwmRight);

void enableWallFollow();

void disableWallFollow();


void dispersion(signed int *pwmLeft, signed int *pwmRight);

void enableDispersion();

void disableDispersion();



#ifdef __cplusplus
} // extern "C"
#endif

#endif
