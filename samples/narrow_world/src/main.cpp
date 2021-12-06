/*
 * Test with:
 *
 * west build -b native_posix samples/narrow_world/ -t run
 */

#include <zephyr.h>
#include <sys/printk.h>

int64_t test_var = INT64_MAX;

int64_t test_fn(void)
{
	return INT64_MAX;
}

void main(void)
{
	uint32_t test;

	test = test_fn();
	test = test_var;

	printk("Narrow World! %u\n", test);
}
