/*
 * Copyright (c) 2011, Thierry Genovese.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * The name of the author may not be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef HAILFIRE_DRIVERS_KLV_HELPER_H
#define HAILFIRE_DRIVERS_KLV_HELPER_H

#include <stdint.h>
#include <map>
#include <vector>

namespace hailfire_drivers
{

typedef uint8_t klv_byte_t;
typedef std::vector<klv_byte_t> klv_byte_vector_t;
typedef std::map<klv_byte_t, klv_byte_vector_t> klv_byte_map_t;

/*!
 * \brief Returns a KLV-encoded vector containing a map's key-value bytes pairs.
 *
 * Returns a KLV-encoded vector of bytes (8-bit unsigned integers) composed (for
 * each pair in the map) with the key (one byte), the length (one byte) and the
 * bytes of the value.
 *
 * Pairs are sorted numerically by key in the output vector.
 *
 * \param keys_values a map of key-value bytes pairs
 */
klv_byte_vector_t encode_klv(klv_byte_map_t const &keys_values);

/*!
 * \brief Returns a map containing a vector's KLV-encoded key-value bytes pairs.
 *
 * Returns a map of key-value bytes pairs with the data found in the given bytes
 * vector.
 *
 * \param bytes a KLV-encoded vector
 */
klv_byte_map_t decode_klv(klv_byte_vector_t const &bytes);

/*!
 * \brief Returns a map resulting from the re-association of separate keys and
 * values.
 *
 * Returns a map of key-value bytes pairs using the map as the source of the keys
 * and lengths, and the vector as the source of the value bytes.
 *
 * \param keys_placeholders a map of key-value bytes pairs
 * \param bytes a bytes vector
 */
klv_byte_map_t reconcile_klv(klv_byte_map_t const &keys_placeholders, klv_byte_vector_t const &bytes);

}

#endif
