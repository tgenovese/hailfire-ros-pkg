#include "hailfire_drivers/klv_helper.h"
#include <gtest/gtest.h>

// The following macros come from: http://stackoverflow.com/questions/1460703

// Using the google test framework, check all elements of two containers
#define EXPECT_ITERABLE_BASE( PREDICATE, REFTYPE, TARTYPE, ref, target) \
    { \
    const REFTYPE& ref_(ref); \
    const TARTYPE& target_(target); \
    REFTYPE::const_iterator refIter = ref_.begin(); \
    TARTYPE::const_iterator tarIter = target_.begin(); \
    unsigned int i = 0; \
    while(refIter != ref_.end()) { \
        if ( tarIter == target_.end() ) { \
            ADD_FAILURE() << #target " has a smaller length than " #ref ; \
            break; \
        } \
        PREDICATE(* refIter, * tarIter) \
            << "Containers " #ref  " (refIter) and " #target " (tarIter)" \
               " differ at index " << i; \
        ++refIter; ++tarIter; ++i; \
    } \
    EXPECT_TRUE( tarIter == target_.end() ) \
        << #ref " has a smaller length than " #target ; \
    }

// Check that all elements of two same-type containers are equal
#define EXPECT_ITERABLE_EQ( TYPE, ref, target) \
    EXPECT_ITERABLE_BASE( EXPECT_EQ, TYPE, TYPE, ref, target )

// Check that all elements of two different-type containers are equal
#define EXPECT_ITERABLE_EQ2( REFTYPE, TARTYPE, ref, target) \
    EXPECT_ITERABLE_BASE( EXPECT_EQ, REFTYPE, TARTYPE, ref, target )


using namespace hailfire_drivers;

TEST(KLVHelper, encodeKLV)
{
  // Input test data
  klv_byte_map_t input_pairs;
  input_pairs[0x01].push_back(0xDE);
  input_pairs[0x01].push_back(0xAD);
  input_pairs[0x01].push_back(0xBE);
  input_pairs[0x01].push_back(0xEF);
  input_pairs[0x03].push_back(0xF0);
  input_pairs[0x03].push_back(0x0D);
  input_pairs[0x02].push_back(0x00);
  input_pairs[0x02].push_back(0x00);

  // Expected output data
  klv_byte_vector_t ref_bytes;
  ref_bytes.push_back(0x01); // first key
  ref_bytes.push_back(4);    // length of value
  ref_bytes.push_back(0xDE); // value byte
  ref_bytes.push_back(0xAD); // value byte
  ref_bytes.push_back(0xBE); // value byte
  ref_bytes.push_back(0xEF); // value byte
  ref_bytes.push_back(0x02); // second key
  ref_bytes.push_back(2);    // length of value
  ref_bytes.push_back(0x00); // value byte
  ref_bytes.push_back(0x00); // value byte
  ref_bytes.push_back(0x03); // third key
  ref_bytes.push_back(2);    // length of value
  ref_bytes.push_back(0xF0); // value byte
  ref_bytes.push_back(0x0D); // value byte

  // Encode input map
  klv_byte_vector_t bytes = encode_klv(input_pairs);

  EXPECT_ITERABLE_EQ(klv_byte_vector_t, ref_bytes, bytes);
}

TEST(KLVHelper, decodeKLV)
{
  // Input test data
  klv_byte_vector_t input_bytes;
  input_bytes.push_back(0x01); // first key
  input_bytes.push_back(4);    // length of value
  input_bytes.push_back(0xDE); // value byte
  input_bytes.push_back(0xAD); // value byte
  input_bytes.push_back(0xBE); // value byte
  input_bytes.push_back(0xEF); // value byte
  input_bytes.push_back(0x02); // second key
  input_bytes.push_back(2);    // length of value
  input_bytes.push_back(0x00); // value byte
  input_bytes.push_back(0x00); // value byte
  input_bytes.push_back(0x03); // third key
  input_bytes.push_back(2);    // length of value
  input_bytes.push_back(0xF0); // value byte
  input_bytes.push_back(0x0D); // value byte

  // Expected output data
  klv_byte_map_t ref_pairs;
  ref_pairs[0x01].push_back(0xDE);
  ref_pairs[0x01].push_back(0xAD);
  ref_pairs[0x01].push_back(0xBE);
  ref_pairs[0x01].push_back(0xEF);
  ref_pairs[0x03].push_back(0xF0);
  ref_pairs[0x03].push_back(0x0D);
  ref_pairs[0x02].push_back(0x00);
  ref_pairs[0x02].push_back(0x00);

  // Decode input vector
  klv_byte_map_t pairs = decode_klv(input_bytes);

  // Check content pair by pair
  for (klv_byte_map_t::iterator it = ref_pairs.begin(); it != ref_pairs.end(); it++)
  {
    // Check key
    if (pairs.count(it->first) == 0)
    {
      ADD_FAILURE() << "Key " << (unsigned int)it->first << " not found";
      continue;
    }

    // Check value
    klv_byte_vector_t value = pairs[it->first];
    klv_byte_vector_t ref_value = it->second;
    EXPECT_ITERABLE_EQ(klv_byte_vector_t, ref_value, value);
  }
}

TEST(KLVHelper, reconcileKLV)
{
  // Input test pairs
  klv_byte_map_t input_pairs;
  input_pairs[0x01].push_back(0xDE);
  input_pairs[0x01].push_back(0xAD);
  input_pairs[0x01].push_back(0xBE);
  input_pairs[0x01].push_back(0xEF);
  input_pairs[0x03].push_back(0xF0);
  input_pairs[0x03].push_back(0x0D);
  input_pairs[0x02].push_back(0x00);
  input_pairs[0x02].push_back(0x00);

  // Input test bytes: keys and lengths zeroed out
  klv_byte_vector_t input_bytes;
  input_bytes.push_back(0);    // first key
  input_bytes.push_back(0);    // length of value
  input_bytes.push_back(0xDE); // value byte
  input_bytes.push_back(0xAD); // value byte
  input_bytes.push_back(0xBE); // value byte
  input_bytes.push_back(0xEF); // value byte
  input_bytes.push_back(0);    // second key
  input_bytes.push_back(0);    // length of value
  input_bytes.push_back(0xBA); // value byte
  input_bytes.push_back(0xAD); // value byte
  input_bytes.push_back(0);    // third key
  input_bytes.push_back(0);    // length of value
  input_bytes.push_back(0xF0); // value byte
  input_bytes.push_back(0x0D); // value byte

  // Expected output data
  klv_byte_map_t ref_pairs;
  ref_pairs[0x01].push_back(0xDE);
  ref_pairs[0x01].push_back(0xAD);
  ref_pairs[0x01].push_back(0xBE);
  ref_pairs[0x01].push_back(0xEF);
  ref_pairs[0x02].push_back(0xBA);
  ref_pairs[0x02].push_back(0xAD);
  ref_pairs[0x03].push_back(0xF0);
  ref_pairs[0x03].push_back(0x0D);

  // Reconcile sent pairs with received bytes
  klv_byte_map_t pairs = reconcile_klv(input_pairs, input_bytes);

  // Check content pair by pair
  for (klv_byte_map_t::iterator it = ref_pairs.begin(); it != ref_pairs.end(); it++)
  {
    // Check key
    if (pairs.count(it->first) == 0)
    {
      ADD_FAILURE() << "Key " << (unsigned int)it->first << " not found";
      continue;
    }

    // Check value
    klv_byte_vector_t value = pairs[it->first];
    klv_byte_vector_t ref_value = it->second;
    EXPECT_ITERABLE_EQ(klv_byte_vector_t, ref_value, value);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
