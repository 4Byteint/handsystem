#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xda140291, "module_layout" },
	{ 0x188ea314, "jiffies_to_timespec64" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x891bb1b4, "kmalloc_caches" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0xf9a482f9, "msleep" },
	{ 0x718764f0, "vCanCleanup" },
	{ 0xb5b54b34, "_raw_spin_unlock" },
	{ 0x54b1fac6, "__ubsan_handle_load_invalid_value" },
	{ 0x5021bd81, "_raw_write_lock_irqsave" },
	{ 0x8b3033ac, "dma_set_mask" },
	{ 0xbe2f960a, "pci_disable_device" },
	{ 0x56470118, "__warn_printk" },
	{ 0x837b7b09, "__dynamic_pr_debug" },
	{ 0xc5476fa4, "pci_release_regions" },
	{ 0xc6f46339, "init_timer_key" },
	{ 0xa648e561, "__ubsan_handle_shift_out_of_bounds" },
	{ 0x3c3ff9fd, "sprintf" },
	{ 0x15ba50a6, "jiffies" },
	{ 0xeb078aee, "_raw_write_unlock_irqrestore" },
	{ 0x5a8d6745, "vCanRemoveCardChannel" },
	{ 0x679e43d1, "queue_empty" },
	{ 0x25974000, "wait_for_completion" },
	{ 0x6610c28d, "pci_set_master" },
	{ 0xdcb764ad, "memset" },
	{ 0xfaa20ff6, "queue_front" },
	{ 0x1b41a302, "dma_sync_single_for_cpu" },
	{ 0xa14430dc, "vCanDispatchEvent" },
	{ 0x2754ff6a, "pci_iounmap" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x449ad0a7, "memcmp" },
	{ 0xb77cf577, "set_capability_value" },
	{ 0x5c0c34ec, "set_capability_ex_mask" },
	{ 0x9166fada, "strncpy" },
	{ 0xe6cf5658, "queue_wakeup_on_space" },
	{ 0x4b750f53, "_raw_spin_unlock_irq" },
	{ 0x220f6eb0, "queue_pop" },
	{ 0x69dd3b5b, "crc32_le" },
	{ 0x235ea4c1, "calculateCRC32" },
	{ 0x24d273d1, "add_timer" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0x5f181617, "pci_clear_master" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x406f7e3f, "vCanInitData" },
	{ 0xb1342cdb, "_raw_read_lock_irqsave" },
	{ 0xa916b694, "strnlen" },
	{ 0x5f5a4166, "set_capability_ex_value" },
	{ 0xa09c6340, "vCanGetCardInfo2" },
	{ 0x8da6585d, "__stack_chk_fail" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0xb8b9f817, "kmalloc_order_trace" },
	{ 0x8427cc7b, "_raw_spin_lock_irq" },
	{ 0x92997ed8, "_printk" },
	{ 0x6b2dc060, "dump_stack" },
	{ 0xa1a93e61, "dma_map_page_attrs" },
	{ 0xdf2ebb87, "_raw_read_unlock_irqrestore" },
	{ 0x69f38847, "cpu_hwcap_keys" },
	{ 0xcb72a358, "dev_driver_string" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0xef984b52, "pci_unregister_driver" },
	{ 0xac1f3f82, "kmem_cache_alloc_trace" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0xf5e2db06, "set_capability_mask" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0xc3055d20, "usleep_range_state" },
	{ 0x206ebad6, "queue_irq_lock" },
	{ 0x50ac5583, "vCanGetCardInfo" },
	{ 0x37a0cba, "kfree" },
	{ 0x4829a47e, "memcpy" },
	{ 0x8137e77d, "pci_request_regions" },
	{ 0x25418f61, "vCanInit" },
	{ 0x8c2d15ba, "dma_sync_single_for_device" },
	{ 0x4a584d, "__pci_register_driver" },
	{ 0x672182c2, "dma_unmap_page_attrs" },
	{ 0x608741b5, "__init_swait_queue_head" },
	{ 0x55555880, "queue_reinit" },
	{ 0x30372d96, "queue_release" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0xa6257a2f, "complete" },
	{ 0xa429a168, "pci_iomap" },
	{ 0x75c55d35, "vCanAddCardChannel" },
	{ 0xd5975f01, "pci_enable_device" },
	{ 0x4a3ad70e, "wait_for_completion_timeout" },
	{ 0x14b89635, "arm64_const_caps_ready" },
	{ 0xc31db0ce, "is_vmalloc_addr" },
	{ 0xc1514a3b, "free_irq" },
};

MODULE_INFO(depends, "kvcommon");


MODULE_INFO(srcversion, "E8B14086EC2221F3B4A69E6");
