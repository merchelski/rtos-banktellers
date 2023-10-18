#ifndef INC_RNG_H_
/*
 * rng.h
 *
 *  Created on: Oct 18, 2023
 *      Author: merchelski
 */

/*  Written in 2019 by David Blackman and Sebastiano Vigna (vigna@acm.org)

To the extent possible under law, the author has dedicated all copyright
and related and neighboring rights to this software to the public domain
worldwide. This software is distributed without any warranty.

See <http://creativecommons.org/publicdomain/zero/1.0/>. */

/* This is xoroshiro128++ 1.0, one of our all-purpose, rock-solid,
   small-state generators. It is extremely (sub-ns) fast and it passes all
   tests we are aware of, but its state space is large enough only for
   mild parallelism.

   For generating just floating-point numbers, xoroshiro128+ is even
   faster (but it has a very mild bias, see notes in the comments).

   The state must be seeded so that it is not everywhere zero. If you have
   a 64-bit seed, we suggest to seed a splitmix64 generator and use its
   output to fill s. */

#include <stdint.h>

#define SEED1 (2552996413275464428ULL)
#define SEED2 (9599274583936742465ULL)

/* Generate next random number in the sequence generated from seeds. */
uint64_t generate_random_u64(void);
uint64_t rand_range(uint64_t start_inclusive, uint64_t end_inclusive);

#define INC_RNG_H_
#endif /* INC_RNG_H_ */
