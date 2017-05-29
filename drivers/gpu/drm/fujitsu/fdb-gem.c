/*
 * drivers/gpu/drm/omapdrm/fdb_gem.c
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Rob Clark <rob.clark@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <linux/spinlock.h>
#include <linux/shmem_fs.h>
#include <linux/iommu.h>
#include <linux/dma-buf.h>
#include <asm/dma-iommu.h>
#include <linux/completion.h>

#include "fdb-drm.h"
#include "drm_fb_helper.h"

/*
 * GEM buffer object implementation.
 */

#define to_fdb_bo(x) container_of(x, struct fdb_gem_object, base)

/* note: we use upper 8 bits of flags for driver-internal flags: */
#define FDB_BO_DMA	0x01000000	/* actually is physically contiguous */
#define FDB_BO_EXT_SYNC	0x02000000	/* externally allocated sync object */
#define FDB_BO_EXT_MEM	0x04000000	/* externally allocated memory */
#define FDB_BO_PREALLOC 0x08000000	/* 1 page allocated behind this */

struct fdb_gem_object {
	struct drm_gem_object base;

	struct list_head mm_list;

	uint32_t flags;

	/** width/height for tiled formats (rounded up to slot boundaries) */
	uint16_t width, height;

	/** roll applied when mapping to DMM */
	uint32_t roll;

	/**
	 * If buffer is allocated physically contiguous, the FDB_BO_DMA flag
	 * is set and the paddr is valid.  Also if the buffer is remapped in
	 * TILER and paddr_cnt > 0, then paddr is valid.  But if you are using
	 * the physical address and FDB_BO_DMA is not set, then you should
	 * be going thru fdb_gem_{get,put}_paddr() to ensure the mapping is
	 * not removed from under your feet.
	 *
	 * Note that FDB_BO_SCANOUT is a hint from userspace that DMA capable
	 * buffer is requested, but doesn't mean that it is.  Use the
	 * FDB_BO_DMA flag to determine if the buffer has a DMA capable
	 * physical address.
	 */
	dma_addr_t paddr;

	/**
	 * # of users of paddr
	 */
	uint32_t paddr_cnt;

	/**
	 * tiler block used when buffer is remapped in DMM/TILER.
	 */
	struct tiler_block *block;

	/**
	 * Array of backing pages, if allocated.  Note that pages are never
	 * allocated for buffers originally allocated from contiguous memory
	 */
	struct page **pages;

	/** addresses corresponding to pages in above array */
	dma_addr_t *addrs;

	/**
	 * Virtual address, if mapped.
	 */
	void *vaddr;

	/**
	 * sync-object allocated on demand (if needed)
	 *
	 * Per-buffer sync-object for tracking pending and completed hw/dma
	 * read and write operations.  The layout in memory is dictated by
	 * the SGX firmware, which uses this information to stall the command
	 * stream if a surface is not ready yet.
	 *
	 * Note that when buffer is used by SGX, the sync-object needs to be
	 * allocated from a special heap of sync-objects.  This way many sync
	 * objects can be packed in a page, and not waste GPU virtual address
	 * space.  Because of this we have to have a fdb_gem_set_sync_object()
	 * API to allow replacement of the syncobj after it has (potentially)
	 * already been allocated.  A bit ugly but I haven't thought of a
	 * better alternative.
	 */
	struct {
		uint32_t write_pending;
		uint32_t write_complete;
		uint32_t read_pending;
		uint32_t read_complete;
	} *sync;

	struct dma_attrs dma_attrs;
	struct completion async_comp;
};

static int get_pages(struct drm_gem_object *obj, struct page ***pages);
static uint64_t mmap_offset(struct drm_gem_object *obj);

/* To deal with userspace mmap'ings of 2d tiled buffers, which (a) are
 * not necessarily pinned in TILER all the time, and (b) when they are
 * they are not necessarily page aligned, we reserve one or more small
 * regions in each of the 2d containers to use as a user-GART where we
 * can create a second page-aligned mapping of parts of the buffer
 * being accessed from userspace.
 *
 * Note that we could optimize slightly when we know that multiple
 * tiler containers are backed by the same PAT.. but I'll leave that
 * for later..
 */
#define NUM_USERGART_ENTRIES 2
struct usergart_entry {
	struct tiler_block *block;	/* the reserved tiler block */
	dma_addr_t paddr;
	struct drm_gem_object *obj;	/* the current pinned obj */
	pgoff_t obj_pgoff;		/* page offset of obj currently
					   mapped in */
};
static struct {
	struct usergart_entry entry[NUM_USERGART_ENTRIES];
	int height;				/* height in rows */
	int height_shift;		/* ilog2(height in rows) */
	int slot_shift;			/* ilog2(width per slot) */
	int stride_pfn;			/* stride in pages */
	int last;				/* index of last used entry */
} *usergart;

/* GEM objects can either be allocated from contiguous memory (in which
 * case obj->filp==NULL), or w/ shmem backing (obj->filp!=NULL).  But non
 * contiguous buffers can be remapped in TILER/DMM if they need to be
 * contiguous... but we don't do this all the time to reduce pressure
 * on TILER/DMM space when we know at allocation time that the buffer
 * will need to be scanned out.
 */
static inline bool is_shmem(struct drm_gem_object *obj)
{
	return obj->filp != NULL;
}

/**
 * shmem buffers that are mapped cached can simulate coherency via using
 * page faulting to keep track of dirty pages
 */
static inline bool is_cached_coherent(struct drm_gem_object *obj)
{
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);
	return is_shmem(obj) &&
		((fdb_obj->flags & FDB_BO_CACHE_MASK) == FDB_BO_CACHED);
}

static DEFINE_SPINLOCK(sync_lock);

/** ensure backing pages are allocated */
static int fdb_gem_attach_pages(struct drm_gem_object *obj)
{
	struct drm_device *dev = obj->dev;
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);
	struct page **pages;
	int npages = obj->size >> PAGE_SHIFT;
	int i, ret;
	dma_addr_t *addrs;

	WARN_ON(fdb_obj->pages);

	/* TODO: __GFP_DMA32 .. but somehow GFP_HIGHMEM is coming from the
	 * mapping_gfp_mask(mapping) which conflicts w/ GFP_DMA32.. probably
	 * we actually want CMA memory for it all anyways..
	 */
	pages = _drm_gem_get_pages(obj, GFP_KERNEL);
	if (IS_ERR(pages)) {
		dev_err(obj->dev->dev,
			"could not get pages: %ld\n", PTR_ERR(pages));
		return PTR_ERR(pages);
	}

	/* for non-cached buffers, ensure the new pages are clean because
	 * DSS, GPU, etc. are not cache coherent:
	 */
	if (fdb_obj->flags & (FDB_BO_WC | FDB_BO_UNCACHED)) {
		addrs = kmalloc(npages * sizeof(*addrs), GFP_KERNEL);
		if (!addrs) {
			ret = -ENOMEM;
			goto free_pages;
		}

		for (i = 0; i < npages; i++) {
			addrs[i] = dma_map_page(dev->dev, pages[i],
					0, PAGE_SIZE, DMA_BIDIRECTIONAL);
		}
	} else {
		addrs = kzalloc(npages * sizeof(*addrs), GFP_KERNEL);
		if (!addrs) {
			ret = -ENOMEM;
			goto free_pages;
		}
	}

	fdb_obj->addrs = addrs;
	fdb_obj->pages = pages;

	return 0;

free_pages:
	_drm_gem_put_pages(obj, pages, true, false);

	return ret;
}

/** release backing pages */
static void fdb_gem_detach_pages(struct drm_gem_object *obj)
{
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);

	/* for non-cached buffers, ensure the new pages are clean because
	 * DSS, GPU, etc. are not cache coherent:
	 */
	if (fdb_obj->flags & (FDB_BO_WC | FDB_BO_UNCACHED)) {
		int i, npages = obj->size >> PAGE_SHIFT;
		for (i = 0; i < npages; i++) {
			dma_unmap_page(obj->dev->dev, fdb_obj->addrs[i],
					PAGE_SIZE, DMA_BIDIRECTIONAL);
		}
	}

	kfree(fdb_obj->addrs);
	fdb_obj->addrs = NULL;

	_drm_gem_put_pages(obj, fdb_obj->pages, true, false);
	fdb_obj->pages = NULL;
}

/* get buffer flags */
uint32_t fdb_gem_flags(struct drm_gem_object *obj)
{
	return to_fdb_bo(obj)->flags;
}

/** get mmap offset */
static uint64_t mmap_offset(struct drm_gem_object *obj)
{
	struct drm_device *dev = obj->dev;

	/* pr_err(" mmap_offset\n"); */

	WARN_ON(!mutex_is_locked(&dev->struct_mutex));

	if (!obj->map_list.map) {
		/* Make it mmapable */
		size_t size = fdb_gem_mmap_size(obj);
		int ret = _drm_gem_create_mmap_offset_size(obj, size);

		if (ret) {
			dev_err(dev->dev, "could not allocate mmap offset\n");
			return 0;
		}
	}

	return (uint64_t)obj->map_list.hash.key << PAGE_SHIFT;
}

uint64_t fdb_gem_mmap_offset(struct drm_gem_object *obj)
{
	uint64_t offset;
	mutex_lock(&obj->dev->struct_mutex);
	offset = mmap_offset(obj);
	mutex_unlock(&obj->dev->struct_mutex);
	return offset;
}

uint64_t fdb_gem_paddr(struct drm_gem_object *obj)
{
	uint64_t paddr;
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);

	mutex_lock(&obj->dev->struct_mutex);
	if (fdb_obj->flags & FDB_BO_PREALLOC)
		paddr = fdb_obj->paddr + SZ_4K;
	else
		paddr = fdb_obj->paddr;
	mutex_unlock(&obj->dev->struct_mutex);

	return paddr;
}



/** get mmap size */
size_t fdb_gem_mmap_size(struct drm_gem_object *obj)
{
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);
	size_t size = obj->size;
	if (fdb_obj->flags & FDB_BO_PREALLOC)
		return size - SZ_4K;

	return size;
}


/* Normal handling for the case of faulting in non-tiled buffers */
static int fault_1d(struct drm_gem_object *obj,
		struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);
	unsigned long pfn;
	pgoff_t pgoff;
	struct dma_iommu_mapping *iommu_mapping
		= to_dma_iommu_mapping(obj->dev->dev);

	/* We don't use vmf->pgoff since that has the fake offset: */
	pgoff = ((unsigned long)vmf->virtual_address -
			vma->vm_start) >> PAGE_SHIFT;
			
	if (fdb_obj->flags & FDB_BO_PREALLOC)
		pgoff++; /* everything one page ahead */

	if (fdb_obj->pages) {
		fdb_gem_cpu_sync(obj, pgoff);
		pfn = page_to_pfn(fdb_obj->pages[pgoff]);
	} else {
		BUG_ON(!(fdb_obj->flags & FDB_BO_DMA));
		if (!iommu_mapping)
			pfn = (fdb_obj->paddr >> PAGE_SHIFT) + pgoff;
		else
			pfn = (iommu_iova_to_phys(
					iommu_mapping->domain,
					fdb_obj->paddr) >> PAGE_SHIFT) + pgoff;
	}

	/* pr_dbg("Inserting %p pfn %lx, pa %lx", vmf->virtual_address,
			pfn, pfn << PAGE_SHIFT); */

	return vm_insert_mixed(vma, (unsigned long)vmf->virtual_address, pfn);
}



/**
 * fdb_gem_fault		-	pagefault handler for GEM objects
 * @vma: the VMA of the GEM object
 * @vmf: fault detail
 *
 * Invoked when a fault occurs on an mmap of a GEM managed area. GEM
 * does most of the work for us including the actual map/unmap calls
 * but we need to do the actual page work.
 *
 * The VMA was set up by GEM. In doing so it also ensured that the
 * vma->vm_private_data points to the GEM object that is backing this
 * mapping.
 */
int fdb_gem_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct drm_gem_object *obj = vma->vm_private_data;
/*	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj); */
	struct drm_device *dev = obj->dev;
	struct page **pages;
	int ret;

	/* Make sure we don't parallel update on a fault, nor move or remove
	 * something from beneath our feet
	 */
	mutex_lock(&dev->struct_mutex);

	/* if a shmem backed object, make sure we have pages attached now */
	ret = get_pages(obj, &pages);
	if (ret)
		goto fail;

	/* where should we do corresponding put_pages().. we are mapping
	 * the original page, rather than thru a GART, so we can't rely
	 * on eviction to trigger this.  But munmap() or all mappings should
	 * probably trigger put_pages()?
	 */

	ret = fault_1d(obj, vma, vmf);


fail:
	mutex_unlock(&dev->struct_mutex);
	switch (ret) {
	case 0:
	case -ERESTARTSYS:
	case -EINTR:
		return VM_FAULT_NOPAGE;
	case -ENOMEM:
		return VM_FAULT_OOM;
	default:
		return VM_FAULT_SIGBUS;
	}
}

/** We override mainly to fix up some of the vm mapping flags.. */
int fdb_gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int ret;

	pr_debug("fdb_gem_mmap: %p %p\n", filp, vma);

	ret = drm_gem_mmap(filp, vma);
	if (ret) {
		pr_err("mmap failed: %d\n", ret);
		return ret;
	}

	return fdb_gem_mmap_obj(vma->vm_private_data, vma);
}

int fdb_gem_mmap_obj(struct drm_gem_object *obj,
		struct vm_area_struct *vma)
{
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);

	vma->vm_flags &= ~VM_PFNMAP;
	vma->vm_flags |= VM_MIXEDMAP;

	if (fdb_obj->flags & FDB_BO_WC)
		vma->vm_page_prot =
			pgprot_writecombine(vm_get_page_prot(vma->vm_flags));
	else if (fdb_obj->flags & FDB_BO_UNCACHED)
		vma->vm_page_prot =
			pgprot_noncached(vm_get_page_prot(vma->vm_flags));
	else {
		/*
		 * We do have some private objects, at least for scanout buffers
		 * on hardware without DMM/TILER.  But these are allocated write
		 * combine
		 */
		if (WARN_ON(!obj->filp))
			return -EINVAL;

		/*
		 * Shunt off cached objs to shmem file so they have their own
		 * address_space (so unmap_mapping_range does what we want,
		 * in particular in the case of mmap'd dmabufs)
		 */
		fput(vma->vm_file);
		vma->vm_pgoff = 0;
		vma->vm_file  = get_file(obj->filp);

		vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	}

	return 0;
}


/**
 * fdb_gem_dumb_create	-	create a dumb buffer
 * @drm_file: our client file
 * @dev: our device
 * @args: the requested arguments copied from userspace
 *
 * Allocate a buffer suitable for use for a frame buffer of the
 * form described by user space. Give userspace a handle by which
 * to reference it.
 */
int fdb_gem_dumb_create(struct drm_file *file, struct drm_device *dev,
		struct drm_mode_create_dumb *args)
{
	u32 gsize;

	args->pitch = 0;

	pr_debug("fdb_gem_dumb_create: pitch: %d, w: %d, bpp: %d, h: %d\n",
		args->pitch, args->width, args->bpp, args->height);

	/* in case someone tries to feed us a completely bogus stride: */
	args->pitch = align_pitch(args->pitch, args->width, args->bpp);
	args->size = PAGE_ALIGN(args->pitch * args->height);

	gsize = args->size;

	return fdb_gem_new_handle(dev, file, gsize,
		FDB_BO_SCANOUT | FDB_BO_WC | FDB_BO_PREALLOC, &args->handle);
}

/**
 * fdb_gem_dumb_destroy	-	destroy a dumb buffer
 * @file: client file
 * @dev: our DRM device
 * @handle: the object handle
 *
 * Destroy a handle that was created via fdb_gem_dumb_create.
 */
int fdb_gem_dumb_destroy(struct drm_file *file, struct drm_device *dev,
		uint32_t handle)
{
	/* No special work needed, drop the reference and see what falls out */
	return drm_gem_handle_delete(file, handle);
}

/**
 * fdb_gem_dumb_map	-	buffer mapping for dumb interface
 * @file: our drm client file
 * @dev: drm device
 * @handle: GEM handle to the object (from dumb_create)
 *
 * Do the necessary setup to allow the mapping of the frame buffer
 * into user memory. We don't have to do much here at the moment.
 */
int fdb_gem_dumb_map_offset(struct drm_file *file, struct drm_device *dev,
		uint32_t handle, uint64_t *offset)
{
	struct drm_gem_object *obj;
	int ret = 0;

	/* GEM does all our handle to object mapping */
	obj = drm_gem_object_lookup(dev, file, handle);
	if (obj == NULL) {
		ret = -ENOENT;
		goto fail;
	}

	*offset = fdb_gem_mmap_offset(obj);

	drm_gem_object_unreference_unlocked(obj);

fail:
	return ret;
}

/* Set scrolling position.  This allows us to implement fast scrolling
 * for console.
 *
 * Call only from non-atomic contexts.
 */
int fdb_gem_roll(struct drm_gem_object *obj, uint32_t roll)
{
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);
	uint32_t npages = obj->size >> PAGE_SHIFT;
	int ret = 0;

	if (roll > npages) {
		dev_err(obj->dev->dev, "invalid roll: %d\n", roll);
		return -EINVAL;
	}

	fdb_obj->roll = roll;

	mutex_lock(&obj->dev->struct_mutex);

	/* if we aren't mapped yet, we don't need to do anything */
	if (fdb_obj->block) {
		struct page **pages;
		ret = get_pages(obj, &pages);
		if (ret)
			goto fail;
/*		ret = tiler_pin(fdb_obj->block, pages, npages, roll, true);
		if (ret)
			dev_err(obj->dev->dev, "could not repin: %d\n", ret); */
	}

fail:
	mutex_unlock(&obj->dev->struct_mutex);

	return ret;
}

/* Sync the buffer for CPU access.. note pages should already be
 * attached, ie. fdb_gem_get_pages()
 */
void fdb_gem_cpu_sync(struct drm_gem_object *obj, int pgoff)
{
	struct drm_device *dev = obj->dev;
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);

	if (is_cached_coherent(obj) && fdb_obj->addrs[pgoff]) {
		dma_unmap_page(dev->dev, fdb_obj->addrs[pgoff],
				PAGE_SIZE, DMA_BIDIRECTIONAL);
		fdb_obj->addrs[pgoff] = 0;
	}
}

/* sync the buffer for DMA access */
void fdb_gem_dma_sync(struct drm_gem_object *obj,
		enum dma_data_direction dir)
{
	struct drm_device *dev = obj->dev;
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);
	int i, npages = obj->size >> PAGE_SHIFT;
	struct page **pages = fdb_obj->pages;
	bool dirty = false;

	if (!is_cached_coherent(obj))
		return;

	for (i = 0; i < npages; i++)
		if (!fdb_obj->addrs[i]) {
			fdb_obj->addrs[i] = dma_map_page(dev->dev, pages[i], 0,
						PAGE_SIZE, DMA_BIDIRECTIONAL);
			dirty = true;
		}

	if (dirty) {
		unmap_mapping_range(obj->filp->f_mapping, 0,
					fdb_gem_mmap_size(obj), 1);
	}
}

void fdb_gem_put_width_height(struct drm_gem_object *obj,
		uint16_t w, uint16_t h)
{
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);

	fdb_obj->width = w;
	fdb_obj->height = h;
}

void fdb_gem_get_width_height(struct drm_gem_object *obj,
		uint16_t *w, uint16_t *h)
{
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);

	*w = fdb_obj->width;
	*h = fdb_obj->height;
}

/* Get physical address for DMA.. if 'remap' is true, and the buffer is not
 * already contiguous, remap it to pin in physically contiguous memory.. (ie.
 * map in TILER)
 */
int fdb_gem_get_paddr(struct drm_gem_object *obj,
		phys_addr_t *paddr, bool remap)
{
/*	struct fdb_drm_private *priv = obj->dev->dev_private; */
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);
	int ret = 0;

	mutex_lock(&obj->dev->struct_mutex);

	if (fdb_obj->flags & FDB_BO_DMA) {
		if (fdb_obj->flags & FDB_BO_PREALLOC)
			*paddr = fdb_obj->paddr + SZ_4K;
		else
			*paddr = fdb_obj->paddr;
	} else {
		dev_err(obj->dev->dev, "%s failed, no FDB_BO_DMA flag\n", __func__); 
		ret = -EINVAL;
		goto fail;
	}

fail:
	mutex_unlock(&obj->dev->struct_mutex);

	return ret;
}

/* Release physical address, when DMA is no longer being performed.. this
 * could potentially unpin and unmap buffers from TILER
 */
int fdb_gem_put_paddr(struct drm_gem_object *obj)
{
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);
	int ret = 0;

	mutex_lock(&obj->dev->struct_mutex);
	if (fdb_obj->paddr_cnt > 0) {
		fdb_obj->paddr_cnt--;
		if (fdb_obj->paddr_cnt == 0)
			fdb_obj->block = NULL;
	}

	mutex_unlock(&obj->dev->struct_mutex);
	return ret;
}


/* acquire pages when needed (for example, for DMA where physically
 * contiguous buffer is not required
 */
static int get_pages(struct drm_gem_object *obj, struct page ***pages)
{
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);
	int ret = 0;

	if (is_shmem(obj) && !fdb_obj->pages) {
		ret = fdb_gem_attach_pages(obj);
		if (ret) {
			dev_err(obj->dev->dev, "could not attach pages\n");
			return ret;
		}
	}

	/* TODO: even phys-contig.. we should have a list of pages? */
	*pages = fdb_obj->pages;

	return 0;
}

/* if !remap, and we don't have pages backing, then fail, rather than
 * increasing the pin count (which we don't really do yet anyways,
 * because we don't support swapping pages back out).  And 'remap'
 * might not be quite the right name, but I wanted to keep it working
 * similarly to fdb_gem_get_paddr().  Note though that mutex is not
 * aquired if !remap (because this can be called in atomic ctxt),
 * but probably fdb_gem_get_paddr() should be changed to work in the
 * same way.  If !remap, a matching fdb_gem_put_pages() call is not
 * required (and should not be made).
 */
int fdb_gem_get_pages(struct drm_gem_object *obj, struct page ***pages,
		bool remap)
{
	int ret;
	if (!remap) {
		struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);
		if (!fdb_obj->pages)
			return -ENOMEM;
		*pages = fdb_obj->pages;
		return 0;
	}
	mutex_lock(&obj->dev->struct_mutex);
	ret = get_pages(obj, pages);
	mutex_unlock(&obj->dev->struct_mutex);
	return ret;
}

/* release pages when DMA no longer being performed */
int fdb_gem_put_pages(struct drm_gem_object *obj)
{
	/* do something here if we dynamically attach/detach pages.. at
	 * least they would no longer need to be pinned if everyone has
	 * released the pages..
	 */
	return 0;
}

/* Get kernel virtual address for CPU access.. this more or less only
 * exists for fdb_fbdev.  This should be called with struct_mutex
 * held.
 */
void *fdb_gem_vaddr(struct drm_gem_object *obj)
{
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);
	WARN_ON(!mutex_is_locked(&obj->dev->struct_mutex));
	if (!fdb_obj->vaddr) {
		struct page **pages;
		int ret = get_pages(obj, &pages);
		if (ret)
			return ERR_PTR(ret);
		fdb_obj->vaddr = vmap(pages, obj->size >> PAGE_SHIFT,
				VM_MAP, pgprot_writecombine(PAGE_KERNEL));
	}
	if (fdb_obj->flags & FDB_BO_PREALLOC)
		return fdb_obj->vaddr + SZ_4K;

	return fdb_obj->vaddr;
}

#ifdef CONFIG_PM
/* re-pin objects in DMM in resume path: */
int fdb_gem_resume(struct device *dev)
{
	return 0;
}
#endif

#ifdef CONFIG_DEBUG_FS
void fdb_gem_describe(struct drm_gem_object *obj, struct seq_file *m)
{
	struct drm_device *dev = obj->dev;
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);
	uint64_t off = 0;

	WARN_ON(!mutex_is_locked(&dev->struct_mutex));

	if (obj->map_list.map)
		off = (uint64_t)obj->map_list.hash.key;

	seq_printf(m, "%08x: %2d (%2d) %08llx %08llx (%2d) %p %4d",
			fdb_obj->flags, obj->name,
			obj->refcount.refcount.counter,
			off, (u64)fdb_obj->paddr, fdb_obj->paddr_cnt,
			fdb_obj->vaddr, fdb_obj->roll);

	seq_printf(m, " %d\n", obj->size);
}

void fdb_gem_describe_objects(struct list_head *list, struct seq_file *m)
{
	struct fdb_gem_object *fdb_obj;
	int count = 0;
	size_t size = 0;

	list_for_each_entry(fdb_obj, list, mm_list) {
		struct drm_gem_object *obj = &fdb_obj->base;
		seq_puts(m, "   ");
		fdb_gem_describe(obj, m);
		count++;
		size += obj->size;
	}

	seq_printf(m, "Total %d objects, %zu bytes\n", count, size);
}
#endif

/* Buffer Synchronization:
 */

struct fdb_gem_sync_waiter {
	struct list_head list;
	struct fdb_gem_object *fdb_obj;
	enum fdb_gem_op op;
	uint32_t read_target, write_target;
	/* notify called w/ sync_lock held */
	void (*notify)(void *arg);
	void *arg;
};

/* list of fdb_gem_sync_waiter.. the notify fxn gets called back when
 * the read and/or write target count is achieved which can call a user
 * callback (ex. to kick 3d and/or 2d), wakeup blocked task (prep for
 * cpu access), etc.
 */
static LIST_HEAD(waiters);

static inline bool is_waiting(struct fdb_gem_sync_waiter *waiter)
{
	struct fdb_gem_object *fdb_obj = waiter->fdb_obj;
	if ((waiter->op & FDB_GEM_READ) &&
			(fdb_obj->sync->read_complete < waiter->read_target))
		return true;
	if ((waiter->op & FDB_GEM_WRITE) &&
			(fdb_obj->sync->write_complete < waiter->write_target))
		return true;
	return false;
}

/* macro for sync debug.. */
#define SYNCDBG 0

#define SYNC(fmt, ...) do { if (SYNCDBG) \
		pr_err("%s:%d: "fmt"\n", \
				__func__, __LINE__, ##__VA_ARGS__); \
	} while (0)


static void sync_op_update(void)
{
	struct fdb_gem_sync_waiter *waiter, *n;
	list_for_each_entry_safe(waiter, n, &waiters, list) {
		if (!is_waiting(waiter)) {
			list_del(&waiter->list);
			SYNC("notify: %p", waiter);
			waiter->notify(waiter->arg);
			kfree(waiter);
		}
	}
}

static inline int sync_op(struct drm_gem_object *obj,
		enum fdb_gem_op op, bool start)
{
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);
	int ret = 0;

	spin_lock(&sync_lock);

	if (!fdb_obj->sync) {
		fdb_obj->sync = kzalloc(sizeof(*fdb_obj->sync), GFP_ATOMIC);
		if (!fdb_obj->sync) {
			ret = -ENOMEM;
			goto unlock;
		}
	}

	if (start) {
		if (op & FDB_GEM_READ)
			fdb_obj->sync->read_pending++;
		if (op & FDB_GEM_WRITE)
			fdb_obj->sync->write_pending++;
	} else {
		if (op & FDB_GEM_READ)
			fdb_obj->sync->read_complete++;
		if (op & FDB_GEM_WRITE)
			fdb_obj->sync->write_complete++;
		sync_op_update();
	}

unlock:
	spin_unlock(&sync_lock);

	return ret;
}

/* it is a bit lame to handle updates in this sort of polling way, but
 * in case of PVR, the GPU can directly update read/write complete
 * values, and not really tell us which ones it updated.. this also
 * means that sync_lock is not quite sufficient.  So we'll need to
 * do something a bit better when it comes time to add support for
 * separate 2d hw..
 */
void fdb_gem_op_update(void)
{
	spin_lock(&sync_lock);
	sync_op_update();
	spin_unlock(&sync_lock);
}

/* mark the start of read and/or write operation */
int fdb_gem_op_start(struct drm_gem_object *obj, enum fdb_gem_op op)
{
	return sync_op(obj, op, true);
}

int fdb_gem_op_finish(struct drm_gem_object *obj, enum fdb_gem_op op)
{
	return sync_op(obj, op, false);
}

static DECLARE_WAIT_QUEUE_HEAD(sync_event);

static void sync_notify(void *arg)
{
	struct task_struct **waiter_task = arg;
	*waiter_task = NULL;
	wake_up_all(&sync_event);
}

int fdb_gem_op_sync(struct drm_gem_object *obj, enum fdb_gem_op op)
{
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);
	int ret = 0;
	if (fdb_obj->sync) {
		struct task_struct *waiter_task = current;
		struct fdb_gem_sync_waiter *waiter =
				kzalloc(sizeof(*waiter), GFP_KERNEL);

		if (!waiter)
			return -ENOMEM;

		waiter->fdb_obj = fdb_obj;
		waiter->op = op;
		waiter->read_target = fdb_obj->sync->read_pending;
		waiter->write_target = fdb_obj->sync->write_pending;
		waiter->notify = sync_notify;
		waiter->arg = &waiter_task;

		spin_lock(&sync_lock);
		if (is_waiting(waiter)) {
			SYNC("waited: %p", waiter);
			list_add_tail(&waiter->list, &waiters);
			spin_unlock(&sync_lock);
			ret = wait_event_interruptible(sync_event,
					(waiter_task == NULL));
			spin_lock(&sync_lock);
			if (waiter_task) {
				SYNC("interrupted: %p", waiter);
				/* we were interrupted */
				list_del(&waiter->list);
				waiter_task = NULL;
			} else {
				/* freed in sync_op_update() */
				waiter = NULL;
			}
		}
		spin_unlock(&sync_lock);

		kfree(waiter);
	}
	return ret;
}

/* call fxn(arg), either synchronously or asynchronously if the op
 * is currently blocked..  fxn() can be called from any context
 *
 * (TODO for now fxn is called back from whichever context calls
 * fdb_gem_op_update().. but this could be better defined later
 * if needed)
 *
 * TODO more code in common w/ _sync()..
 */
int fdb_gem_op_async(struct drm_gem_object *obj, enum fdb_gem_op op,
		void (*fxn)(void *arg), void *arg)
{
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);
	if (fdb_obj->sync) {
		struct fdb_gem_sync_waiter *waiter =
				kzalloc(sizeof(*waiter), GFP_ATOMIC);

		if (!waiter)
			return -ENOMEM;

		waiter->fdb_obj = fdb_obj;
		waiter->op = op;
		waiter->read_target = fdb_obj->sync->read_pending;
		waiter->write_target = fdb_obj->sync->write_pending;
		waiter->notify = fxn;
		waiter->arg = arg;

		spin_lock(&sync_lock);
		if (is_waiting(waiter)) {
			SYNC("waited: %p", waiter);
			list_add_tail(&waiter->list, &waiters);
			spin_unlock(&sync_lock);
			return 0;
		}

		spin_unlock(&sync_lock);
	}

	/* no waiting.. */
	fxn(arg);

	return 0;
}

/* special API so PVR can update the buffer to use a sync-object allocated
 * from it's sync-obj heap.  Only used for a newly allocated (from PVR's
 * perspective) sync-object, so we overwrite the new syncobj w/ values
 * from the already allocated syncobj (if there is one)
 */
int fdb_gem_set_sync_object(struct drm_gem_object *obj, void *syncobj)
{
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);
	int ret = 0;

	spin_lock(&sync_lock);

	if ((fdb_obj->flags & FDB_BO_EXT_SYNC) && !syncobj) {
		/* clearing a previously set syncobj */
		syncobj = kmemdup(fdb_obj->sync, sizeof(*fdb_obj->sync),
				  GFP_ATOMIC);
		if (!syncobj) {
			ret = -ENOMEM;
			goto unlock;
		}
		fdb_obj->flags &= ~FDB_BO_EXT_SYNC;
		fdb_obj->sync = syncobj;
	} else if (syncobj && !(fdb_obj->flags & FDB_BO_EXT_SYNC)) {
		/* replacing an existing syncobj */
		if (fdb_obj->sync) {
			memcpy(syncobj, fdb_obj->sync, sizeof(*fdb_obj->sync));
			kfree(fdb_obj->sync);
		}
		fdb_obj->flags |= FDB_BO_EXT_SYNC;
		fdb_obj->sync = syncobj;
	}

unlock:
	spin_unlock(&sync_lock);
	return ret;
}

int fdb_gem_init_object(struct drm_gem_object *obj)
{
	return -EINVAL;          /* unused */
}

/* don't call directly.. called from GEM core when it is time to actually
 * free the object..
 */
void fdb_gem_free_object(struct drm_gem_object *obj)
{
	struct drm_device *dev = obj->dev;
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);

	WARN_ON(!mutex_is_locked(&dev->struct_mutex));

	list_del(&fdb_obj->mm_list);

	if (obj->map_list.map)
		drm_gem_free_mmap_offset(obj);

	/* this means the object is still pinned.. which really should
	 * not happen.  I think..
	 */
	WARN_ON(fdb_obj->paddr_cnt > 0);

	/* don't free externally allocated backing memory */
	if (!(fdb_obj->flags & FDB_BO_EXT_MEM)) {
		if (fdb_obj->pages)
			fdb_gem_detach_pages(obj);

		if (!is_shmem(obj)) {
			dma_free_attrs(dev->dev, obj->size,
					fdb_obj->vaddr, fdb_obj->paddr, &fdb_obj->dma_attrs);
		} else if (fdb_obj->vaddr) {
			vunmap(fdb_obj->vaddr);
		}
	}

	/* don't free externally allocated syncobj */
	if (!(fdb_obj->flags & FDB_BO_EXT_SYNC))
		kfree(fdb_obj->sync);

	drm_gem_object_release(obj);

	kfree(obj);
}

u32 fdb_fb_get_dma_buf(struct drm_device *dev, struct drm_gem_object *obj)
{
	struct fdb_gem_object *fdb_obj = to_fdb_bo(obj);
	int fd = -1;

	if (dev->driver->gem_prime_export) {
		struct dma_buf *buf = NULL;
		buf = dev->driver->gem_prime_export(dev, &fdb_obj->base, O_RDWR);
		if (buf) {
			fd = dma_buf_fd(buf, O_RDWR);
			if (fd >= 0)
				drm_gem_object_reference(obj);
			else
				dma_buf_put(buf);
		}
		else
			dev_err(dev->dev, "%s: unable to map\n", __func__);
	} else
		dev_err(dev->dev, "No gem_prime_export\n");

	return fd;
}
EXPORT_SYMBOL_GPL(fdb_fb_get_dma_buf);


/* convenience method to construct a GEM buffer object, and userspace handle */
int fdb_gem_new_handle(struct drm_device *dev, struct drm_file *file,
		u32 gsize, uint32_t flags, uint32_t *handle)
{
	struct drm_gem_object *obj;
	int ret;

	obj = fdb_gem_new(dev, gsize, flags);
	if (!obj)
		return -ENOMEM;

	ret = drm_gem_handle_create(file, obj, handle);
	if (ret) {
		drm_gem_object_release(obj);
		/* TODO isn't there a dtor to call? just copying i915 */
		kfree(obj);
		return ret;
	}

	/* drop reference from allocate - handle holds it now */
	drm_gem_object_unreference_unlocked(obj);

	return 0;
}

/* GEM buffer object constructor */
struct drm_gem_object *fdb_gem_new(struct drm_device *dev,
		u32 gsize, uint32_t flags)
{
	struct fdb_drm_private *priv = dev->dev_private;
	struct fdb_gem_object *fdb_obj;
	struct drm_gem_object *obj = NULL;
	size_t size;
	int ret;

	size = PAGE_ALIGN(gsize);
	if (flags & FDB_BO_PREALLOC)
		size += SZ_4K;

	fdb_obj = kzalloc(sizeof(*fdb_obj), GFP_KERNEL);
	if (!fdb_obj)
		goto fail;

	list_add(&fdb_obj->mm_list, &priv->obj_list);

	obj = &fdb_obj->base;

	init_dma_attrs(&fdb_obj->dma_attrs);

	/* without this, IOMMU fails to map properly */
	if (to_dma_iommu_mapping(dev->dev))
		dma_set_attr(DMA_ATTR_FORCE_CONTIGUOUS, &fdb_obj->dma_attrs);

	dma_set_attr(DMA_ATTR_WRITE_COMBINE, &fdb_obj->dma_attrs);

	if (flags & FDB_BO_SCANOUT) {
		/* attempt to allocate contiguous memory */
		fdb_obj->vaddr = dma_alloc_attrs(dev->dev, size,
			&fdb_obj->paddr, GFP_KERNEL, &fdb_obj->dma_attrs);
		if (fdb_obj->vaddr)
			flags |= FDB_BO_DMA;
		else
			dev_err(dev->dev, "%s: failed to allocate dma-able memory\n", __func__);

	}

	fdb_obj->flags = flags;

	if (flags & (FDB_BO_DMA | FDB_BO_EXT_MEM))
		ret = drm_gem_private_object_init(dev, obj, size);
	else
		ret = drm_gem_object_init(dev, obj, size);

	if (ret)
		goto fail;

	obj->driver_private = &fdb_obj->async_comp;
	return obj;

fail:
	if (obj)
		fdb_gem_free_object(obj);

	return NULL;
}

/* init/cleanup.. if DMM is used, we need to set some stuff up.. */
void fdb_gem_init(struct drm_device *dev)
{
/*	struct fdb_drm_private *priv = dev->dev_private; */

	usergart = kcalloc(3, sizeof(*usergart), GFP_KERNEL);
}

void fdb_gem_deinit(struct drm_device *dev)
{
	/* I believe we can rely on there being no more outstanding GEM
	 * objects which could depend on usergart/dmm at this point.
	 */
	kfree(usergart);
}
