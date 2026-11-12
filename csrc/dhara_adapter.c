#include <stdint.h>
#include "dhara/map.h"

#define XCORE_DHARA_PAGE_SIZE 2048
#define XCORE_DHARA_LOG2_PAGE_SIZE 11
#define XCORE_DHARA_LOG2_PAGES_PER_BLOCK 6
#define XCORE_DHARA_BLOCK_COUNT 1024
#define XCORE_DHARA_GC_RATIO 4

static struct dhara_nand xcore_nand;
static struct dhara_map xcore_map;
static uint8_t xcore_page_buffer[XCORE_DHARA_PAGE_SIZE];
static int xcore_initialized;

void xcore_dhara_map_init(void)
{
	xcore_nand.log2_page_size = XCORE_DHARA_LOG2_PAGE_SIZE;
	xcore_nand.log2_ppb = XCORE_DHARA_LOG2_PAGES_PER_BLOCK;
	xcore_nand.num_blocks = XCORE_DHARA_BLOCK_COUNT;

	dhara_map_init(&xcore_map, &xcore_nand, xcore_page_buffer,
		       XCORE_DHARA_GC_RATIO);
	xcore_map.count = 0;
	xcore_initialized = 1;
}

int xcore_dhara_map_resume(int32_t *error)
{
	dhara_error_t err = DHARA_E_NONE;

	if (!xcore_initialized)
		xcore_dhara_map_init();

	const int ret = dhara_map_resume(&xcore_map, &err);
	if (error)
		*error = (int32_t)err;
	return ret;
}

int xcore_dhara_map_provision(uint32_t sector, const uint8_t *data, int32_t *error)
{
	dhara_error_t err = DHARA_E_NONE;
	int ret;

	if (!xcore_initialized || !data) {
		if (error)
			*error = DHARA_E_CORRUPT;
		return -1;
	}

	ret = dhara_map_write(&xcore_map, sector, data, &err);
	if (!ret)
		ret = dhara_map_sync(&xcore_map, &err);
	if (error)
		*error = (int32_t)err;
	return ret;
}

uint32_t xcore_dhara_map_capacity(void)
{
	return xcore_initialized ? dhara_map_capacity(&xcore_map) : 0;
}

int xcore_dhara_map_read(uint32_t sector, uint8_t *data, int32_t *error)
{
	dhara_error_t err = DHARA_E_NONE;
	const int ret = dhara_map_read(&xcore_map, sector, data, &err);
	if (error)
		*error = (int32_t)err;
	return ret;
}

int xcore_dhara_map_write(uint32_t sector, const uint8_t *data,
			  int32_t *error)
{
	dhara_error_t err = DHARA_E_NONE;
	const int ret = dhara_map_write(&xcore_map, sector, data, &err);
	if (error)
		*error = (int32_t)err;
	return ret;
}

int xcore_dhara_map_trim(uint32_t sector, int32_t *error)
{
	dhara_error_t err = DHARA_E_NONE;
	const int ret = dhara_map_trim(&xcore_map, sector, &err);
	if (error)
		*error = (int32_t)err;
	return ret;
}

int xcore_dhara_map_sync(int32_t *error)
{
	dhara_error_t err = DHARA_E_NONE;
	const int ret = dhara_map_sync(&xcore_map, &err);
	if (error)
		*error = (int32_t)err;
	return ret;
}
