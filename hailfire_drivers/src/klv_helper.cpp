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

#include "hailfire_drivers/klv_helper.h"

namespace hailfire_drivers
{

klv_byte_vector_t encode_klv(klv_byte_map_t const &keys_values)
{
  unsigned int nb_bytes = 0;
  klv_byte_map_t::const_iterator it;
  for (it = keys_values.begin(); it != keys_values.end(); it++)
  {
    nb_bytes += 2 + (it->second).size();
  }

  klv_byte_vector_t bytes;
  bytes.reserve(nb_bytes);
  for (it = keys_values.begin(); it != keys_values.end(); it++)
  {
    bytes.push_back(it->first);
    klv_byte_t length = 0xFF & (it->second).size();

    bytes.push_back(length);
    for (int i = 0; i < length; ++i)
    {
      bytes.push_back(it->second[i]);
    }
  }
  return bytes;
}

klv_byte_map_t decode_klv(klv_byte_vector_t const &bytes)
{
  typedef enum
  {
    READ_KEY,
    READ_LENGTH,
    READ_VALUE
  } klv_decoder_state_t;

  klv_decoder_state_t state = READ_KEY;

  klv_byte_t key = 0;
  klv_byte_t length = 0;

  klv_byte_map_t keys_values;
  klv_byte_vector_t::const_iterator it;
  for (it = bytes.begin(); it < bytes.end(); it++)
  {
    switch (state)
    {
      case READ_KEY:
        key = *it;
        state = READ_LENGTH;
        break;
      case READ_LENGTH:
        length = *it;
        if (length != 0)
        {
          state = READ_VALUE;
          keys_values[key].reserve(length);
        }
        else
        {
          state = READ_KEY;
        }
        break;
      case READ_VALUE:
        keys_values[key].push_back(*it);
        if (keys_values[key].size() == length)
        {
          state = READ_KEY;
        }
        break;
    }
  }

  return keys_values;
}

klv_byte_map_t reconcile_klv(klv_byte_map_t const &keys_placeholders, klv_byte_vector_t const &bytes)
{
  typedef enum
  {
    READ_KEY,
    READ_LENGTH,
    READ_VALUE
  } klv_reconciler_state_t;

  klv_reconciler_state_t state = READ_KEY;

  klv_byte_t key = 0;
  klv_byte_t length = 0;

  klv_byte_map_t keys_values;
  klv_byte_vector_t::const_iterator it;
  klv_byte_map_t::const_iterator keys_length_it = keys_placeholders.begin();
  for (it = bytes.begin(); it < bytes.end(); it++)
  {
    switch (state)
    {
      case READ_KEY:
        key = keys_length_it->first;
        state = READ_LENGTH;
        break;
      case READ_LENGTH:
        length = 0xFF & (keys_length_it->second).size();
        keys_length_it++;
        if (length != 0)
        {
          state = READ_VALUE;
          keys_values[key].reserve(length);
        }
        else
        {
          state = READ_KEY;
        }
        break;
      case READ_VALUE:
        keys_values[key].push_back(*it);
        if (keys_values[key].size() == length)
        {
          state = READ_KEY;
        }
        break;
    }
  }

  return keys_values;
}

}
