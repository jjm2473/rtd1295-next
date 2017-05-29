

#ifndef __VPU_DRV_H__
#define __VPU_DRV_H__

#define JDI_IOCTL_MAGIC  'V'

#define JDI_IOCTL_ALLOCATE_PHYSICAL_MEMORY	_IO(JDI_IOCTL_MAGIC, 0)
#define JDI_IOCTL_FREE_PHYSICALMEMORY		_IO(JDI_IOCTL_MAGIC, 1)
#define JDI_IOCTL_WAIT_INTERRUPT		_IO(JDI_IOCTL_MAGIC, 2)
#define JDI_IOCTL_SET_CLOCK_GATE		_IO(JDI_IOCTL_MAGIC, 3)
#define JDI_IOCTL_RESET				_IO(JDI_IOCTL_MAGIC, 4)
#define JDI_IOCTL_GET_INSTANCE_POOL		_IO(JDI_IOCTL_MAGIC, 5)
#define JDI_IOCTL_GET_RESERVED_VIDEO_MEMORY_INFO _IO(JDI_IOCTL_MAGIC, 8)

#endif


/* for userland IOCTL data strcut and
 * it's data length have to same as struct ms_buffer
 */
typedef struct ms_buffer {
	uint32_t size;
	uint64_t phys_addr;
	uint32_t *base;		/* kernel logical address in use kernel */
	uint32_t *virt_addr;	/* virtual user space address */
	uint32_t drm_gem_handle;
} ms_buffer;
