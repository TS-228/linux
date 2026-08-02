/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ION private kernel interface
 *
 * Copyright (C) 2011 Google, Inc.
 */

#ifndef _ION_PRIV_H
#define _ION_PRIV_H

#include <linux/kref.h>
#include <linux/idr.h>
#include <linux/rbtree.h>
#include <linux/types.h>

#include "ion.h"

struct ion_buffer *ion_handle_buffer(struct ion_handle *handle);

/**
 * struct ion_client - a process/hw block local address space
 * @node:		node in the tree of all clients
 * @dev:		backpointer to ion device
 * @handles:		an rb tree of all the handles in this client
 * @idr:		an idr space for allocating handle ids
 * @lock:		lock protecting the tree of handles
 * @name:		used for debugging
 * @display_name:	used for debugging (unique version of @name)
 * @display_serial:	used for debugging (to make display_name unique)
 * @task:		used for debugging
 * @pid:		pid of creating process
 */
struct ion_client {
	struct rb_node node;
	struct ion_device *dev;
	struct rb_root handles;
	struct idr idr;
	struct mutex lock;
	const char *name;
	char *display_name;
	int display_serial;
	struct task_struct *task;
	pid_t pid;
};

/**
 * struct ion_handle - a client local reference to a buffer
 * @ref:		reference count
 * @client:		back pointer to the client the buffer resides in
 * @buffer:		pointer to the buffer
 * @node:		node in the client's handle rbtree
 * @kmap_cnt:		count of times this client has mapped to kernel
 * @id:			client-unique id allocated by client->idr
 */
struct ion_handle {
	struct kref ref;
	struct ion_client *client;
	struct ion_buffer *buffer;
	struct rb_node node;
	unsigned int kmap_cnt;
	int id;
};

struct ion_device *ion_device_create(long (*custom_ioctl)
				     (struct ion_client *client,
				      unsigned int cmd,
				      unsigned long arg));
void ion_device_destroy(struct ion_device *dev);
void ion_device_add_heap(struct ion_device *dev, struct ion_heap *heap);

struct ion_client *ion_client_create(struct ion_device *dev,
				     const char *name);
void ion_client_destroy(struct ion_client *client);

/**
 * struct ion_platform_heap - platform heap description
 * @type:    heap type
 * @id:      heap id
 * @name:    heap name
 * @base:    base address for carveout heaps
 * @size:    size for carveout heaps
 * @align:   alignment requirement
 * @priv:    private platform data
 */
struct ion_platform_heap {
	enum ion_heap_type type;
	unsigned int id;
	const char *name;
	phys_addr_t base;
	size_t size;
	unsigned int alignment;
	void *priv;
};

struct ion_platform_data {
	int nr;
	struct ion_platform_heap *heaps;
};

struct ion_handle *ion_handle_get_by_id_nolock(struct ion_client *client,
					       int id);
int ion_handle_put_nolock(struct ion_handle *handle);

int ion_heap_pages_zero(struct page *page, size_t size, pgprot_t pgprot);
void ion_pages_sync_for_device(struct device *dev, struct page *page,
			       size_t size, enum dma_data_direction dir);

struct ion_heap *ion_heap_create(struct ion_platform_heap *heap_data);
void ion_heap_destroy(struct ion_heap *heap);

#if !defined(CONFIG_ION_RTK)
struct ion_device *ion_internal_device(void);
#endif

#endif /* _ION_PRIV_H */
