#ifndef __GPU_DRM_FDB_DRM_H__
#define __GPU_DRM_FDB_DRM_H__

#include <linux/module.h>
#include <linux/types.h>
#include <linux/completion.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <video/fdb.h>

#include <uapi/drm/fdb-drm.h>
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
#include <linux/kds.h>
#endif

#define FDB_BO_SCANOUT	0x00000001	/* scanout capable (phys contiguous) */
#define FDB_BO_CACHE_MASK 0x00000006	/* cache type mask, see cache modes */
#define FDB_BO_TILED_MASK 0x00000f00	/* tiled mapping msk, see tiled modes */

/* cache modes */
#define FDB_BO_CACHED	0x00000000	/* default */
#define FDB_BO_WC	0x00000002	/* write-combine */
#define FDB_BO_UNCACHED	0x00000004	/* strongly-ordered (uncached) */

#define FDB_BO_PREALLOC 0x08000000	/* 1 page allocated behind this */

/* this should probably be in drm-core to standardize amongst drivers */
#define DRM_ROTATE_0	0
#define DRM_ROTATE_90	1
#define DRM_ROTATE_180	2
#define DRM_ROTATE_270	3
#define DRM_REFLECT_X	4
#define DRM_REFLECT_Y	5


/* clockwise rotation angle */
enum fdb_rotation_angle {
	FDB_ROT_0   = 0,
	FDB_ROT_90  = 1,
	FDB_ROT_180 = 2,
	FDB_ROT_270 = 3,
};

struct fdb_dss_cpr_coefs {
	s16 rr, rg, rb;
	s16 gr, gg, gb;
	s16 br, bg, bb;
};

struct fdb_overlay_info {
	u32 paddr;
	u32 p_uv_addr;  /* for NV12 format */
	u16 screen_width;
	u16 width;
	u16 height;
/*	enum fdb_color_mode color_mode; */
	u8 rotation;
	bool mirror;

	u16 pos_x;
	u16 pos_y;
	u16 out_width;	/* if 0, out_width == width */
	u16 out_height;	/* if 0, out_height == height */
	u8 global_alpha;
	u8 pre_mult_alpha;
	u8 zorder;
};

struct fdb_overlay {
	struct kobject kobj;
	struct list_head list;

	/* static fields */
	const char *name;
/*	enum fdb_plane id;
	enum fdb_color_mode supported_modes;
	enum fdb_overlay_caps caps; */

	/* dynamic fields */
/*	struct fdb_overlay_manager *manager; */

	/*
	 * The following functions do not block:
	 *
	 * is_enabled
	 * set_overlay_info
	 * get_overlay_info
	 *
	 * The rest of the functions may block and cannot be called from
	 * interrupt context
	 */

	int (*enable)(struct fdb_overlay *ovl);
	int (*disable)(struct fdb_overlay *ovl);
	bool (*is_enabled)(struct fdb_overlay *ovl);

/*	int (*set_manager)(struct fdb_overlay *ovl,
		struct fdb_overlay_manager *mgr); */
	int (*unset_manager)(struct fdb_overlay *ovl);

	int (*set_overlay_info)(struct fdb_overlay *ovl,
			struct fdb_overlay_info *info);
	void (*get_overlay_info)(struct fdb_overlay *ovl,
			struct fdb_overlay_info *info);

	int (*wait_for_go)(struct fdb_overlay *ovl);

	struct f_fdb_child *(*get_device)(struct fdb_overlay *ovl);
};

struct fdb_overlay_manager_info {
	u32 default_color;

/*	enum fdb_dss_trans_key_type trans_key_type; */
	u32 trans_key;
	bool trans_enabled;

	bool partial_alpha_enabled;

	bool cpr_enable;
	struct fdb_dss_cpr_coefs cpr_coefs;
};



/* parameters which describe (unrotated) coordinates of scanout within a fb: */
struct fdb_drm_window {
	uint32_t rotation;
	int32_t  crtc_x, crtc_y; /* signed because can be offscreen */
	uint32_t crtc_w, crtc_h;
	uint32_t src_x, src_y;
	uint32_t src_w, src_h;
};

/* Once GO bit is set, we can't make further updates to shadowed registers
 * until the GO bit is cleared.  So various parts in the kms code that need
 * to update shadowed registers queue up a pair of callbacks, pre_apply
 * which is called before setting GO bit, and post_apply that is called
 * after GO bit is cleared.  The crtc manages the queuing, and everyone
 * else goes thru fdb_crtc_apply() using these callbacks so that the
 * code which has to deal w/ GO bit state is centralized.
 */
struct fdb_drm_apply {
	struct list_head pending_node, queued_node;
	bool queued;
	void (*pre_apply)(struct fdb_drm_apply *apply);
	void (*post_apply)(struct fdb_drm_apply *apply);
};

/* For transiently registering for different DSS irqs that various parts
 * of the KMS code need during setup/configuration.  We these are not
 * necessarily the same as what drm_vblank_get/put() are requesting, and
 * the hysteresis in drm_vblank_put() is not necessarily desirable for
 * internal housekeeping related irq usage.
 */
struct fdb_drm_irq {
	struct list_head node;
	uint32_t irqmask;
	bool registered;
	void (*irq)(struct fdb_drm_irq *irq, uint32_t irqstatus);
};

#define MAX_DRM_ELEMENTS 8

struct fdb_drm_private {
	int id;
	struct f_fdb_child child;
	struct drm_device *dev; /* drm_device that owns us */

	unsigned int num_crtcs;
	struct drm_crtc *crtcs[MAX_DRM_ELEMENTS];
	/* there is a child framebuffer bound for each CRTC */
	struct f_fdb_child *bound[MAX_DRM_ELEMENTS];

	unsigned int num_planes;
	struct drm_plane *planes[MAX_DRM_ELEMENTS];

	unsigned int num_encoders;
	struct drm_encoder *encoders[MAX_DRM_ELEMENTS];

	unsigned int num_connectors;
	struct drm_connector *connectors[MAX_DRM_ELEMENTS];

	struct drm_fb_helper *fbdev;

	struct workqueue_struct *wq;

	/* list of GEM objects: */
	struct list_head obj_list;

	/* properties: */
	struct drm_property *rotation_prop;
	struct drm_property *zorder_prop;

	/* irq handling: */
	struct list_head irq_list;    /* list of fdb_drm_irq */
	uint32_t vblank_mask;         /* irq bits set for userspace vblank */
/*	struct fdb_drm_irq error_handler; */
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	struct kds_callback kds_cb;
	struct kds_callback kds_sync_shared_buf_cb;
	/* Cache for flip resources used to avoid kmalloc on each page flip */
	struct kmem_cache *page_flip_slab;
	atomic_t nr_flips_in_flight;
	wait_queue_head_t wait_for_flips;
#endif
};

#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
struct fdb_drm_flip_resource {
	/* This is the kds set associated to the dma_buf we want to flip */
	struct kds_resource_set *kds_res_set;
	struct drm_crtc *crtc;
	struct drm_framebuffer *cur_fb;
	struct drm_framebuffer *old_fb;
	struct work_struct unref_work;
	void *dmabuf;
};
#endif

struct fdb_crtc {
	struct drm_crtc base;
	struct drm_plane *plane;

	struct f_fdb_child *head;

	char name[10];
	int pipe;

	struct drm_pending_vblank_event *event;

	struct fdb_video_timings timings;
	bool enabled;
	bool full_update;
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	spinlock_t current_displaying_lock;
	struct kds_resource_set *old_kds_res_set;
	struct drm_framebuffer *displaying_fb;
#endif
};

#define to_fdb_crtc(x) container_of(x, struct fdb_crtc, base)

struct f_fdb_child *bound_child_from_drm_crtc(
			struct fdb_drm_private *priv, struct drm_crtc *crtc);

struct f_fdb_child *bound_child_from_drm_framebuffer(
		struct fdb_drm_private *priv, struct drm_framebuffer *fb);

void fdb_drm_crtc_set_head(struct drm_crtc *crtc, struct f_fdb_child *child);

struct drm_framebuffer *fdb_fb_init(struct drm_device *dev,
		struct drm_mode_fb_cmd2 *mode_cmd, struct drm_gem_object **bos);
struct drm_framebuffer *fdb_fb_create(struct drm_device *dev,
		struct drm_file *file, struct drm_mode_fb_cmd2 *mode_cmd);
struct drm_fb_helper *fdb_fbdev_init(struct drm_device *dev);
void fdb_fbdev_free(struct drm_device *dev);
struct drm_gem_object *fdb_fb_bo(struct drm_framebuffer *fb, int p);
int fdb_fb_replace(struct drm_framebuffer *a,
		struct drm_framebuffer *b, void *arg,
		void (*unpin)(void *arg, struct drm_gem_object *bo));
phys_addr_t get_linear_addr(struct drm_framebuffer *fb);

void fdb_gem_put_width_height(struct drm_gem_object *obj,
		uint16_t w, uint16_t h);
void fdb_gem_get_width_height(struct drm_gem_object *obj,
		uint16_t *w, uint16_t *h);
int fdb_gem_get_paddr(struct drm_gem_object *obj,
					phys_addr_t *paddr, bool remap);
size_t fdb_gem_mmap_size(struct drm_gem_object *obj);
void fdb_gem_dma_sync(struct drm_gem_object *obj,
		enum dma_data_direction dir);
struct drm_gem_object *fdb_gem_new(struct drm_device *dev,
		u32 gsize, uint32_t flags);
void *fdb_gem_vaddr(struct drm_gem_object *obj);
void fdb_gem_cpu_sync(struct drm_gem_object *obj, int pgoff);
int fdb_gem_new_handle(struct drm_device *dev, struct drm_file *file,
		u32 gsize, uint32_t flags, uint32_t *handle);
int fdb_gem_put_paddr(struct drm_gem_object *obj);
uint64_t fdb_gem_paddr(struct drm_gem_object *obj);


int fdb_gem_mmap_obj(struct drm_gem_object *obj,
		struct vm_area_struct *vma);
void fdb_gem_init(struct drm_device *dev);
void fdb_gem_deinit(struct drm_device *dev);
int fdb_gem_op_async(struct drm_gem_object *obj, enum fdb_gem_op op,
		void (*fxn)(void *arg), void *arg);
int fdb_gem_mmap(struct file *filp, struct vm_area_struct *vma);

int fdb_gem_init_object(struct drm_gem_object *obj);
void fdb_gem_free_object(struct drm_gem_object *obj);
int fdb_gem_dumb_create(struct drm_file *file, struct drm_device *dev,
		struct drm_mode_create_dumb *args);
int fdb_gem_dumb_destroy(struct drm_file *file, struct drm_device *dev,
		uint32_t handle);
int fdb_gem_dumb_map_offset(struct drm_file *file, struct drm_device *dev,
		uint32_t handle, uint64_t *offset);
int fdb_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf);
void fdb_gem_describe(struct drm_gem_object *obj, struct seq_file *m);
void fdb_gem_describe_objects(struct list_head *list, struct seq_file *m);
void fdb_gem_op_update(void);
int fdb_gem_op_start(struct drm_gem_object *obj, enum fdb_gem_op op);
int fdb_gem_op_finish(struct drm_gem_object *obj, enum fdb_gem_op op);
int fdb_gem_op_sync(struct drm_gem_object *obj, enum fdb_gem_op op);
uint64_t fdb_gem_mmap_offset(struct drm_gem_object *obj);

uint32_t fdb_fb_get_formats(uint32_t *pixel_formats,
		uint32_t max_formats);

struct drm_crtc *fdb_crtc_init(struct drm_device *dev,
		struct drm_plane *plane, int channel, int id);
struct drm_plane *fdb_plane_init(struct drm_device *dev,
		int id, bool private_plane);
int fdb_plane_mode_set(struct drm_plane *plane,
		struct drm_crtc *crtc, struct drm_framebuffer *fb,
		int crtc_x, int crtc_y,
		unsigned int crtc_w, unsigned int crtc_h,
		uint32_t src_x, uint32_t src_y,
		uint32_t src_w, uint32_t src_h,
		void (*fxn)(void *), void *arg);
int fdb_plane_set_property(struct drm_plane *plane,
		struct drm_property *property, uint64_t val);

struct drm_connector *fdb_connector_init(struct drm_device *dev,
		int connector_type, struct f_fdb_child *fdb,
		struct drm_encoder *encoder);

struct drm_encoder *fdb_encoder_init(struct drm_device *dev,
		struct f_fdb_child *child);

int fdb_encoder_update(struct drm_encoder *encoder,
		struct fdb_video_timings *timings);

int fdb_encoder_set_enabled(struct drm_encoder *encoder, bool enabled);
int fdb_plane_dpms(struct drm_plane *plane, int mode);

int fdb_crtc_apply(struct drm_crtc *crtc,
		struct fdb_drm_apply *apply);
void fdb_crtc_call_encoder_update(struct drm_crtc *crtc);
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
void fdb_crtc_page_flip_cb(void *cb1, void *cb2);
#endif

void copy_timings_drm_to_fdb(struct drm_crtc *crtc,
	struct fdb_video_timings *timings, struct drm_display_mode *mode);
void copy_timings_fdb_to_drm(struct drm_display_mode *mode,
		struct fdb_video_timings *timings);

void fdb_plane_install_properties(struct drm_plane *plane,
		struct drm_mode_object *obj);

void *fdb_fb_get_physical_start(struct drm_framebuffer *fb);


struct drm_gem_object *fdb_gem_prime_import(struct drm_device *dev,
		struct dma_buf *buffer);
struct dma_buf *fdb_gem_prime_export(struct drm_device *dev,
		struct drm_gem_object *obj, int flags);

int fdb_gem_get_pages(struct drm_gem_object *obj, struct page ***pages,
		bool remap);

int fdb_gem_put_pages(struct drm_gem_object *obj);


/* remove these once drm core helpers are merged */
struct page **_drm_gem_get_pages(struct drm_gem_object *obj, gfp_t gfpmask);
void _drm_gem_put_pages(struct drm_gem_object *obj, struct page **pages,
		bool dirty, bool accessed);
int _drm_gem_create_mmap_offset_size(struct drm_gem_object *obj, size_t size);

u32 fdb_fb_get_dma_buf(struct drm_device *dev, struct drm_gem_object *bo);


static inline int align_pitch(int pitch, int width, int bpp)
{
	int bytespp = (bpp + 7) / 8;
	/* in case someone tries to feed us a completely bogus stride: */
	pitch = max(pitch, width * bytespp);
	return ALIGN(pitch, 8 * bytespp);
}

static inline int objects_lookup(struct drm_device *dev,
		struct drm_file *filp, uint32_t pixel_format,
		struct drm_gem_object **bos, uint32_t *handles)
{
	int i, n = drm_format_num_planes(pixel_format);

	for (i = 0; i < n; i++) {
		bos[i] = drm_gem_object_lookup(dev, filp, handles[i]);
		if (!bos[i])
			goto fail;

	}

	return 0;

fail:
	while (--i > 0)
		drm_gem_object_unreference_unlocked(bos[i]);

	return -ENOENT;
}

static inline struct completion *fdb_gem_get_comp(struct drm_gem_object *obj)
{
	return (struct completion *)obj->driver_private;
}

#endif
