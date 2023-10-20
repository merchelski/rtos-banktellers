/*
 * rng.c
 *
 *  Created on: Oct 18, 2023
 *      Author: merchelski
 */

/*  Written in 2019 by David Blackman and Sebastiano Vigna (vigna@acm.org)

To the extent possible under law, the author has dedicated all copyright
and related and neighboring rights to this software to the public domain
worldwide. This software is distributed without any warranty.

See <http://creativecommons.org/publicdomain/zero/1.0/>. */

#include <stdint.h>
#include "rng.h"

static inline uint64_t rotl(const uint64_t x, int k)
{
	return (x << k) | (x >> (64 - k));
}

// setting the seeds
static uint64_t s[2] = {SEED1, SEED2};

uint64_t generate_random_u64(void)
{
	const uint64_t s0 = s[0];
	uint64_t s1 = s[1];
	const uint64_t result = rotl(s0 + s1, 17) + s0;

	s1 ^= s0;
	s[0] = rotl(s0, 49) ^ s1 ^ (s1 << 21); // a, b
	s[1] = rotl(s1, 28); // c

	return result;
}

uint64_t rand_range(uint64_t start_inclusive, uint64_t end_inclusive)
{
	uint64_t rand_num;
	do
	{
		rand_num = start_inclusive + generate_random_u64() / ((uint64_t)(-1) / (end_inclusive - start_inclusive + 1) + 1);
	}
	while(rand_num > end_inclusive);

	return rand_num;
}

