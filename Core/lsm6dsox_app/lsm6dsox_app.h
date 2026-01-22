/*
 * lsm6dsox_app.h
 *
 *  Created on: 7 дек. 2025 г.
 *      Author: Dmitrij
 */

#ifndef LSM6DSOX_APP_LSM6DSOX_APP_H_
#define LSM6DSOX_APP_LSM6DSOX_APP_H_

#ifndef LSM6DSOX_APP_H
#define LSM6DSOX_APP_H

#include "lsm6dsox.h"

#ifdef __cplusplus
extern "C" {
#endif

// Возвращает 0 при успехе, <0 при ошибке
int LSM6DSOX_ReadAcceleration(LSM6DSOX_Object_t *pObj, LSM6DSOX_Axes_t *acc);
int LSM6DSOX_ReadGyroscope(LSM6DSOX_Object_t *pObj, LSM6DSOX_Axes_t *gyro);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DSOX_APP_H */

#endif /* LSM6DSOX_APP_LSM6DSOX_APP_H_ */
