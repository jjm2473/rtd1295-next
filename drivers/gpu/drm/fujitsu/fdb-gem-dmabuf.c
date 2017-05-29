/*
 * drivers/gpu/drm/omapdrm/fdb_gem_dmabuf.c
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

#include "fdb-drm.h"

#include <linux/dma-buf.h>
#include <asm/dma-iommu.h>
#include <linux/iommu.h>

static struct sg_table *fdb_gem_map_dma_buf(
		struct dma_buf_attachment *attachment,
		enum dma_data_direction dir)
{
	struct drm_gem_object *obj = attachment->dmabuf->priv;
	struct sg_table *sg;
	struct dma_iommu_mapping *mapping;
	int ret;
	phys_addr_t paddr, real_paddr;

	sg = kzalloc(sizeof(*sg), GFP_KERNEL);
	if (!sg)
		return ERR_PTR(-ENOMEM);

	/* camera, etc, need physically contiguous.. but we need a
	 * better way to know this..
	 */
	ret = fdb_gem_get_paddr(obj, &paddr, true);
	if (ret)
		goto out;

	ret = sg_alloc_table(sg, 1, GFP_KERNEL);
	if (ret)
		goto out;

	mapping = to_dma_iommu_mapping(obj->dev->dev);
	if (mapping) {
		real_paddr = iommu_iova_to_phys(mapping->domain, paddr);
		paddr = real_paddr;
	}

	sg_init_table(sg->sgl, 1);
	sg_dma_len(sg->sgl) = obj->size;
	sg_set_page(sg->sgl, pfn_to_page(PFN_DOWN(paddr)), obj->size, 0);
	sg_dma_address(sg->sgl) = paddr;

	pr_debug("%s %d paddr 0x%llx\n", __func__, __LINE__, (u64)paddr);

	/* this should be after _get_paddr() to ensure we have pages attached */
	fdb_gem_dma_sync(obj, dir);

	return sg;
out:
	kfree(sg);
	return ERR_PTR(ret);
}

static void fdb_gem_unmap_dma_buf(struct dma_buf_attachment *attachment,
		struct sg_table *sg, enum dma_data_direction dir)
{
	struct drm_gem_object *obj = attachment->dmabuf->priv;
	fdb_gem_put_paddr(obj);
	sg_free_table(sg);
	kfree(sg);
}

static void fdb_gem_dmabuf_release(struct dma_buf *buffer)
{
	struct drm_gem_object *obj = buffer->priv;
#ifdef CONFIG_DMA_SHARED_BUFFER_USES_KDS
	struct drm_crtc *crtc;

	list_for_each_entry(crtc, &obj->dev->mode_config.crtc_list, head) {
		struct fdb_crtc *fdb_crtc = to_fdb_crtc(crtc);
		unsigned long flags;

		spin_lock_irqsave(&fdb_crtc->current_displaying_lock, flags);

		if (fdb_crtc->displaying_fb &&
				obj == fdb_fb_bo(fdb_crtc->displaying_fb, 0) && fdb_crtc->old_kds_res_set) {
			kds_resource_set_release(&fdb_crtc->old_kds_res_set);
			fdb_crtc->old_kds_res_set = NULL;
		}

		spin_unlock_irqrestore(&fdb_crtc->current_displaying_lock, flags);
	}
#endif

	drm_gem_object_unreference_unlocked(obj);
}


static int fdb_gem_dmabuf_begin_cpu_access(struct dma_buf *buffer,
		size_t start, size_t len, enum dma_data_direction dir)
{
	struct drm_gem_object *obj = buffer->priv;
	struct page **pages;

	/* make sure we have the pages: */
	return fdb_gem_get_pages(obj, &pages, true);
}

static void fdb_gem_dmabuf_end_cpu_access(struct dma_buf *buffer,
		size_t start, size_t len, enum dma_data_direction dir)
{
	struct drm_gem_object *obj = buffer->priv;
	fdb_gem_put_pages(obj);
}


static void *fdb_gem_dmabuf_kmap_atomic(struct dma_buf *buffer,
		unsigned long page_num)
{
	struct drm_gem_object *obj = buffer->priv;
	struct page **pages;
	fdb_gem_get_pages(obj, &pages, false);
	fdb_gem_cpu_sync(obj, page_num);
	return kmap_atomic(pages[page_num]);
}

static void fdb_gem_dmabuf_kunmap_atomic(struct dma_buf *buffer,
		unsigned long page_num, void *addr)
{
	kunmap_atomic(addr);
}

static void *fdb_gem_dmabuf_kmap(struct dma_buf *buffer,
		unsigned long page_num)
{
	struct drm_gem_object *obj = buffer->priv;
	struct page **pages;
	fdb_gem_get_pages(obj, &pages, false);
	fdb_gem_cpu_sync(obj, page_num);
	return kmap(pages[page_num]);
}

static void fdb_gem_dmabuf_kunmap(struct dma_buf *buffer,
		unsigned long page_num, void *addr)
{
	struct drm_gem_object *obj = buffer->priv;
	struct page **pages;
	fdb_gem_get_pages(obj, &pages, false);
	kunmap(pages[page_num]);
}

/*
 * TODO maybe we can split up drm_gem_mmap to avoid duplicating
 * some here.. or at least have a drm_dmabuf_mmap helper.
 */
static int fdb_gem_dmabuf_mmap(struct dma_buf *buffer,
		struct vm_area_struct *vma)
{
	struct drm_gem_object *obj = buffer->priv;
	int ret = 0;

	if (WARN_ON(!obj->filp))
		return -EINVAL;

	/* Check for valid size. */
	if (fdb_gem_mmap_size(obj) < vma->vm_end - vma->vm_start) {
		ret = -EINVAL;
		goto out_unlock;
	}

	if (!obj->dev->driver->gem_vm_ops) {
		ret = -EINVAL;
		goto out_unlock;
	}

	vma->vm_flags |= VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP;
	vma->vm_ops = obj->dev->driver->gem_vm_ops;
	vma->vm_private_data = obj;
	vma->vm_page_prot =
			pgprot_writecombine(vm_get_page_prot(vma->vm_flags));

	/* Take a ref for this mapping of the object, so that the fault
	 * handler can dereference the mmap offset's pointer to the object.
	 * This reference is cleaned up by the corresponding vm_close
	 * (which should happen whether the vma was created by this call, or
	 * by a vm_open due to mremap or partial unmap or whatever).
	 */
	vma->vm_ops->open(vma);

out_unlock:

	return fdb_gem_mmap_obj(obj, vma);
}

struct dma_buf_ops fdb_dmabuf_ops = {
		.map_dma_buf = fdb_gem_map_dma_buf,
		.unmap_dma_buf = fdb_gem_unmap_dma_buf,
		.release = fdb_gem_dmabuf_release,
		.begin_cpu_access = fdb_gem_dmabuf_begin_cpu_access,
		.end_cpu_access = fdb_gem_dmabuf_end_cpu_access,
		.kmap_atomic = fdb_gem_dmabuf_kmap_atomic,
		.kunmap_atomic = fdb_gem_dmabuf_kunmap_atomic,
		.kmap = fdb_gem_dmabuf_kmap,
		.kunmap = fdb_gem_dmabuf_kunmap,
		.mmap = fdb_gem_dmabuf_mmap,
};

struct dma_buf *fdb_gem_prime_export(struct drm_device *dev,
		struct drm_gem_object *obj, int flags)
{
	return dma_buf_export(obj, &fdb_dmabuf_ops, obj->size, flags);
}

struct drm_gem_object *fdb_gem_prime_import(struct drm_device *dev,
		struct dma_buf *buffer)
{
	struct drm_gem_object *obj;

	/* is this one of own objects? */
	if (buffer->ops == &fdb_dmabuf_ops) {
		obj = buffer->priv;
		/* is it from our device? */
		if (obj->dev == dev) {
			/*
			 * Importing dmabuf exported from out own gem increases
			 * refcount on gem itself instead of f_count of dmabuf.
			 */
			drm_gem_object_reference(obj);
			return obj;
		}
	}

	/*
	 * TODO add support for importing buffers from other devices..
	 * for now we don't need this but would be nice to add eventually
	 */
	return ERR_PTR(-EINVAL);
}
