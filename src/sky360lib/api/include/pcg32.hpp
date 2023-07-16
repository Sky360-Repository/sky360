#pragma once

namespace sky360lib
{
	class Pcg32 final
	{
	public:
		Pcg32()
		{
			for (size_t i{0}; i < TABLE_SIZE; ++i)
				fixed_table[i] = fast_rt() % TABLE_SIZE;
		}

		// The actual algorithm
		inline uint32_t fast_rt()
		{
			uint64_t x = mcg_state;
			unsigned count = (unsigned)(x >> 61); // 61 = 64 - 3

			mcg_state = x * MULTIPLIER;
			x ^= x >> 22;
			return (uint32_t)(x >> (22 + count)); // 22 = 32 - 3 - 7
		}

		inline uint32_t fast()
		{
			const uint32_t result = fixed_table[current_pos];
			if (++current_pos >= TABLE_SIZE)
				current_pos = 0;
			return result;
		}

	private:
		static uint32_t const TABLE_SIZE = 32768;
		static uint64_t const MULTIPLIER = 6364136223846793005u;

		uint64_t mcg_state{0xcafef00dd15ea5e5u}; // Must be odd
		uint32_t fixed_table[TABLE_SIZE];
		size_t current_pos{0};
	};
}