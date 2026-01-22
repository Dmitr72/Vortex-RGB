/*
 * lsm6dsox_app.c
 *
 *  Created on: 7 дек. 2025 г.
 *      Author: Dmitrij
 */


#include "lsm6dsox_app.h"
#include "lsm6dsox.h"
#include "custom_mems_conf.h" // для CS/портов, при необходимости
#include <stdio.h>

int LSM6DSOX_ReadAcceleration(LSM6DSOX_Object_t *pObj, LSM6DSOX_Axes_t *acc)
{
  if (pObj == NULL || acc == NULL) return -1;

  // Если акселерометр выключен — попытаться включить
  if (pObj->acc_is_enabled == 0U)
  {
    if (LSM6DSOX_ACC_Enable(pObj) != LSM6DSOX_OK)
    {
      return -2;
    }
    // небольшая задержка, чтобы датчик успел стартовать
    if (pObj->IO.Delay) pObj->IO.Delay(10);
  }

  if (LSM6DSOX_ACC_GetAxes(pObj, acc) == LSM6DSOX_OK)
  {
    return 0;
  }

  return -3;
}

/* Чтение гироскопа */
int LSM6DSOX_ReadGyroscope(LSM6DSOX_Object_t *pObj, LSM6DSOX_Axes_t *gyro)
{
  if (pObj == NULL || gyro == NULL) return -1;

  if (pObj->gyro_is_enabled == 0U)
  {
    if (LSM6DSOX_GYRO_Enable(pObj) != LSM6DSOX_OK)
    {
      return -2;
    }
    if (pObj->IO.Delay) pObj->IO.Delay(10);
  }

  if (LSM6DSOX_GYRO_GetAxes(pObj, gyro) == LSM6DSOX_OK)
  {
    return 0;
  }

  return -3;
}

