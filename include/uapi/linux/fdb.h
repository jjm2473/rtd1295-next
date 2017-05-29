#ifndef __INCLUDE_LINUX_FDB_H__
#define __INCLUDE_LINUX_FDB_H__

/*
 * FDB generic structs userland may want
 */

enum fdb_blit_format {
	/* support for usual color formats */
	FBF_BGR888,
	FBF_ARGB8888,
	FBF_ABGR8888,
	FBF_BGRA8888,
	FBF_RGBA8888,
	FBF_YUV420_INTERLEAVE,
	FBF_YUV420_SEPARATE, /* YUV420 planar */
	FBF_YUV422_SEPARATE, /* YUV422 planar */
	FBF_YUV444_SEPARATE, /* YUV444 planar */
};

#define BLIT_COMP_RING_LEN 8
struct blit_completion_id {
	char c_id;
};

struct b_c_id_ring {
	struct blit_completion_id b_c_id[BLIT_COMP_RING_LEN];
};

/*
 * These are the pieces of interest to the kernel blitter code
 */

struct fdb_blit {
	int id;
	struct blit_completion_id b_c_id[BLIT_COMP_RING_LEN];
	uint64_t src_addr; /* physical ads - note - 64bit when LPAE kernel */
	uint64_t mask_addr; /* physical ads - note - 64bit when LPAE kernel */
	uint64_t dest_addr; /* physical ads - note - 64bit when LPAE kernel */
	uint32_t stride_in;
	uint32_t stride_mask;
	uint32_t stride_out;
	uint32_t width_in; /* 0 = filling, see fill_pixel */
	uint32_t height_in; /* 0 = filling, see fill_pixel */
	uint32_t width_out;
	uint32_t height_out;
	uint32_t bits_per_pixel_in;
	uint32_t bits_per_pixel_out;
	uint32_t fill_pixel; /* only need to set when filling */
	uint32_t rotate; /* 0 = no rotation, 0x0001-0xffff = CCW rotation */
	uint32_t src_blend;
	uint32_t dst_blend;
	uint32_t op_blend;
	uint32_t src_x_offset;
	uint32_t src_y_offset;
	uint32_t mask_x_offset;
	uint32_t mask_y_offset;
	uint32_t dest_x_offset;
	uint32_t dest_y_offset;
	uint32_t format_in;
	uint32_t format_out;
	uint8_t	 blend; /* blend:0 0->no blend, 1->blend */
	uint8_t  async; /* 0 = wait for operation,
					 * BLIT_ASYNC = queue and return,
					 * BLIT_ASYNC_NEED_COMP = queue and retrun(need to call ioctl(DRM_IOCTL_FDB_GET_BLIT_COMP_ONE))
					 */
	uint8_t  source_alpha; /* 0 = 100% opaque, 1 = use fill_pixel alpha */
	uint16_t src_size_w;
	uint16_t src_size_h;
	uint16_t dest_size_w;
	uint16_t dest_size_h;
	uint32_t scale_adjust_w; /* Adjust value for JPEG Codec's quirk of upscaling picture. */
	uint32_t scale_adjust_h; /* These variables are used only on upscaling/downscaling.   */
							 /* e.g. If old(before JPEG decode) height is 1080 and new(after JPEG decode) height
							  *      is 1088, set scale_adjust_h to 8.
							  */
};

#define BLIT_ASYNC 1
#define BLIT_ASYNC_NEED_COMP 2

/*
 * This is passed in from userland using fdb blit blocking ioctl
 */

struct blit_block {
	int length; /* userland must set to sizeof(struct blit_block) */
	struct fdb_blit blit;
	uint64_t paddr_in;
	uint64_t paddr_out;
	uint32_t gem_handle_in;
	uint32_t offset_in; /* offset inside the gem allocation to use */
	uint32_t gem_handle_mask;
	uint32_t offset_mask; /* offset inside the gem allocation to use */
	uint32_t gem_handle_out;
	uint32_t offset_out; /* offset inside the gem allocation to use */
};

/*
 * This for blit blend usage
 */
#define GL_ZERO				0x0
#define GL_ONE				0x1
#define GL_SRC_ALPHA			0x302
#define GL_ONE_MINUS_SRC_ALPHA		0x303
#define GL_DST_ALPHA			0x304
#define GL_ONE_MINUS_DST_ALPHA		0x305

#define GL_FUNC_ADD			0x8006
#define GL_FUNC_SUBTRACT		0x800A
#define GL_FUNC_REVERSE_SUBTRACT	0x800B
#define VG_BLEND_DARKEN			0x2007

#define IOCTL_GET_FB_DMA_BUF _IOWR('m',0xF9, __u32 )

#endif

