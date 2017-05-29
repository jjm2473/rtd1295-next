#ifndef __INCLUDE_UAPI_DRM_FDB_DRM_H__
#define __INCLUDE_UAPI_DRM_FDB_DRM_H__

/* mask of operations: */
enum fdb_gem_op {
	FDB_GEM_READ = 0x01,
	FDB_GEM_WRITE = 0x02,
};

struct drm_fdb_gem_cpu_prep {
	uint32_t handle;		/* buffer handle (in) */
	uint32_t op;			/* mask of omap_gem_op (in) */
};

struct drm_fdb_gem_cpu_fini {
	uint32_t handle;		/* buffer handle (in) */
	uint32_t op;			/* mask of omap_gem_op (in) */
	/* TODO maybe here we pass down info about what regions are touched
	 * by sw so we can be clever about cache ops?  For now a placeholder,
	 * set to zero and we just do full buffer flush..
	 */
	uint32_t nregions;
	uint32_t __pad;
};

struct drm_fdb_gem_bo {
	uint32_t handle;
	uint32_t size;
	void *map_addr;
	uint64_t paddr;
	void *base;
	uint32_t width;
	uint32_t height;
	uint8_t bpp;
	uint32_t pitch;
	uint64_t offset;
};

struct drm_fdb_comp {
	uint32_t gem_handle;
	unsigned int wait_msec;
};

/*
 * These next two sections need to match the list in fdb-drv.c
 * struct drm_ioctl_desc ioctls[]
 */

enum fdb_drm_ioctls {
	DRM_FDB_GEM_CPU_PREP,
	DRM_FDB_GEM_CPU_FINI,
	DRM_FDB_GEM_ALLOC,
	DRM_FDB_IRIS_BLIT_BLOCKING,
	DRM_FDB_GET_FB_PADDR,
	DRM_FDB_GET_BLIT_COMP,
	DRM_FDB_GET_BLIT_COMP_ONE,
	DRM_FDB_SYNC_SHARED_BUF,

	/* always last */
	DRM_FDB_NUM_IOCTLS
};

#define DRM_IOCTL_FDB_GEM_CPU_PREP	 DRM_IOW (DRM_COMMAND_BASE + DRM_FDB_GEM_CPU_PREP, struct drm_fdb_gem_cpu_prep)
#define DRM_IOCTL_FDB_GEM_CPU_FINI	 DRM_IOW (DRM_COMMAND_BASE + DRM_FDB_GEM_CPU_FINI, struct drm_fdb_gem_cpu_fini)
#define DRM_IOCTL_FDB_GEM_ALLOC		 DRM_IOWR(DRM_COMMAND_BASE + DRM_FDB_GEM_ALLOC, struct drm_fdb_gem_bo)
#define DRM_IOCTL_FDB_IRIS_BLIT_BLOCKING DRM_IOWR(DRM_COMMAND_BASE + \
			DRM_FDB_IRIS_BLIT_BLOCKING, struct blit_block)
/* just fills in paddr */
#define DRM_IOCTL_FDB_GET_FB_PADDR      DRM_IOR(DRM_COMMAND_BASE + \
			DRM_FDB_GET_FB_PADDR, struct drm_fdb_gem_bo)
#define DRM_IOCTL_FDB_GET_BLIT_COMP	DRM_IOR(DRM_COMMAND_BASE + \
			DRM_FDB_GET_BLIT_COMP, struct b_c_id_ring)
#define DRM_IOCTL_FDB_GET_BLIT_COMP_ONE	DRM_IOW(DRM_COMMAND_BASE + \
			DRM_FDB_GET_BLIT_COMP_ONE, struct drm_fdb_comp)
#define DRM_IOCTL_FDB_SYNC_SHARED_BUF   DRM_IOW(DRM_COMMAND_BASE + \
			DRM_FDB_SYNC_SHARED_BUF, struct drm_fdb_comp)

#endif

