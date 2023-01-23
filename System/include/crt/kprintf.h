/*
 ******************************************************************************
 * $File    kprintf.h
 * $Brief   Header file of Kernel Print Function.
 * $Created on: Jan 18, 2023
 ******************************************************************************
 *
 * Copyright (c) 2023 Digital Museum. All rights reserved.
 *
 * This software component is licensed by Digital Museum under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 *	https://www.youtube.com/@digitalmuseum6400
 *	https://github.com/KwangHyuk73
 *	e-mail:ponytail2k@gmail.com
 ******************************************************************************
 */


#ifndef INCLUDE_CRT_KPRINTF_H_
#define INCLUDE_CRT_KPRINTF_H_

#define FMTFLG_ZERO     0x01    /*!< \brief Set, if zero padding required */
#define FMTFLG_SIGNED   0x02    /*!< \brief Set, if signed value */
#define FMTFLG_PLUS     0x04    /*!< \brief Set to force sign */
#define FMTFLG_MINUS    0x08    /*!< \brief Set to force left justification */
#define FMTFLG_CAPITAL  0x10    /*!< \brief Set for capital letter digits */

extern int kprintBinary(const char *data, int len);
extern int kprintString(const char *str);
extern int kprintInteger(unsigned long val, unsigned char radix, unsigned char width, unsigned char flags);
extern int kprintf(const char *fmt, ...);

#endif /* INCLUDE_CRT_KPRINTF_H_ */
