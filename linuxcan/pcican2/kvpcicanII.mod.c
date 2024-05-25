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
	{ 0x2d3385d3, "system_wq" },
	{ 0x891bb1b4, "kmalloc_caches" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0x718764f0, "vCanCleanup" },
	{ 0xb5b54b34, "_raw_spin_unlock" },
	{ 0x5021bd81, "_raw_write_lock_irqsave" },
	{ 0xbe2f960a, "pci_disable_device" },
	{ 0xc5476fa4, "pci_release_regions" },
	{ 0xeb078aee, "_raw_write_unlock_irqrestore" },
	{ 0x5a8d6745, "vCanRemoveCardChannel" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0x679e43d1, "queue_empty" },
	{ 0xdcb764ad, "memset" },
	{ 0xfaa20ff6, "queue_front" },
	{ 0xa14430dc, "vCanDispatchEvent" },
	{ 0x2754ff6a, "pci_iounmap" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0xb77cf577, "set_capability_value" },
	{ 0xe6cf5658, "queue_wakeup_on_space" },
	{ 0x220f6eb0, "queue_pop" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x406f7e3f, "vCanInitData" },
	{ 0xb1342cdb, "_raw_read_lock_irqsave" },
	{ 0xa09c6340, "vCanGetCardInfo2" },
	{ 0x8da6585d, "__stack_chk_fail" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0xb8b9f817, "kmalloc_order_trace" },
	{ 0x92997ed8, "_printk" },
	{ 0xdf2ebb87, "_raw_read_unlock_irqrestore" },
	{ 0x69f38847, "cpu_hwcap_keys" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0xef984b52, "pci_unregister_driver" },
	{ 0xac1f3f82, "kmem_cache_alloc_trace" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0x50ac5583, "vCanGetCardInfo" },
	{ 0x37a0cba, "kfree" },
	{ 0x4829a47e, "memcpy" },
	{ 0x8137e77d, "pci_request_regions" },
	{ 0x25418f61, "vCanInit" },
	{ 0x4a584d, "__pci_register_driver" },
	{ 0x7747eb49, "vCanFlushSendBuffer" },
	{ 0x92540fbf, "finish_wait" },
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
	{ 0xc1514a3b, "free_irq" },
};

MODULE_INFO(depends, "kvcommon");


MODULE_INFO(srcversion, "283E5674B096A10E4758D46");
