/*
 * Copyright (C) 2015 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    sys_bitfield bitfields of arbitrary length
 * @ingroup     sys
 * @brief       bitfields of arbitrary length
 * @{
 *
 * @brief       bitfields operations on bitfields of arbitrary length
 *
 * @note        Code taken mostly from
 *              <a href="http://stackoverflow.com/questions/1590893/error-trying-to-define-a-1-024-bit-128-byte-bit-field">
 *              Stackoverflow, User Christoph</a>
 *
 * @author      Oliver Hahm <oliver.hahm@inria.fr>
 */

#ifndef BITFIELD_H
#define BITFIELD_H

#include <limits.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Declare a bitfield of a given size
 *
 * @note    SIZE should be a constant expression this avoids variable length
 *          arrays and problems resulting from being evaluated twice
 */
#define BITFIELD(SIZE, NAME) \
     unsigned char NAME[(SIZE) / CHAR_BIT + ((SIZE) % CHAR_BIT != 0)]

/**
 * @brief   Set the bit to 1
 *
 * @param[in,out] field The bitfield
 * @param[in]     idx   The number of the bit to set
 */
static inline void bf_set(unsigned char field[], size_t idx)
{
    field[idx / CHAR_BIT] |= 1u << (idx % CHAR_BIT);
}

/**
 * @brief   Clear the bit
 *
 * @param[in,out] field The bitfield
 * @param[in]     idx   The number of the bit to clear
 */
static inline void bf_unset(unsigned char field[], size_t idx)
{
    field[idx / CHAR_BIT] &= ~(1u << (idx % CHAR_BIT));
}

/**
 * @brief   Toggle the bit
 *
 * @param[in,out] field The bitfield
 * @param[in]     idx   The number of the bit to toggle
 */
static inline void bf_toggle(unsigned char field[], size_t idx)
{
    field[idx / CHAR_BIT] ^= 1u << (idx % CHAR_BIT);
}

/**
 * @brief  Check if the bet is set
 *
 * @param[in,out] field The bitfield
 * @param[in]     idx   The number of the bit to check
 */
static inline bool bf_isset(unsigned char field[], size_t idx)
{
    return field[idx / CHAR_BIT] & (1u << (idx % CHAR_BIT));
}

#ifdef __cplusplus
}
#endif

/** @} */
#endif /* BITFIELD_H */
