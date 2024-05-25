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
	{ 0xfe2fd6f8, "queue_init" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0x718764f0, "vCanCleanup" },
	{ 0xb5b54b34, "_raw_spin_unlock" },
	{ 0xa02aea3a, "queue_length" },
	{ 0x1caa2ae0, "usb_kill_urb" },
	{ 0x7337a3bd, "kthread_create_on_node" },
	{ 0x5a8d6745, "vCanRemoveCardChannel" },
	{ 0xaad8c7d6, "default_wake_function" },
	{ 0x679e43d1, "queue_empty" },
	{ 0x6782eeca, "queue_push" },
	{ 0x25974000, "wait_for_completion" },
	{ 0xdcb764ad, "memset" },
	{ 0xfaa20ff6, "queue_front" },
	{ 0x87d7787f, "queue_add_wait_for_space" },
	{ 0xa14430dc, "vCanDispatchEvent" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0xaf2aeb47, "usb_deregister" },
	{ 0xb77cf577, "set_capability_value" },
	{ 0xe6cf5658, "queue_wakeup_on_space" },
	{ 0x220f6eb0, "queue_pop" },
	{ 0x8c03d20c, "destroy_workqueue" },
	{ 0x2d10af5, "usb_free_coherent" },
	{ 0x952664c5, "do_exit" },
	{ 0x42160169, "flush_workqueue" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0xe7d54d73, "module_put" },
	{ 0x406f7e3f, "vCanInitData" },
	{ 0xce1928ac, "usb_submit_urb" },
	{ 0xa09c6340, "vCanGetCardInfo2" },
	{ 0x8da6585d, "__stack_chk_fail" },
	{ 0x939b7043, "usb_bulk_msg" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0xb8b9f817, "kmalloc_order_trace" },
	{ 0x92997ed8, "_printk" },
	{ 0x69f38847, "cpu_hwcap_keys" },
	{ 0x8048cc3f, "wake_up_process" },
	{ 0xac1f3f82, "kmem_cache_alloc_trace" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0x50ac5583, "vCanGetCardInfo" },
	{ 0x37a0cba, "kfree" },
	{ 0x4829a47e, "memcpy" },
	{ 0x25418f61, "vCanInit" },
	{ 0x7747eb49, "vCanFlushSendBuffer" },
	{ 0x5d6c6e04, "usb_register_driver" },
	{ 0x10fa71db, "queue_remove_wait_for_space" },
	{ 0x608741b5, "__init_swait_queue_head" },
	{ 0x244ab863, "queue_back" },
	{ 0x55555880, "queue_reinit" },
	{ 0x30372d96, "queue_release" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0xa6257a2f, "complete" },
	{ 0x97e3df8f, "usb_alloc_coherent" },
	{ 0x7f02188f, "__msecs_to_jiffies" },
	{ 0x75c55d35, "vCanAddCardChannel" },
	{ 0x4a3ad70e, "wait_for_completion_timeout" },
	{ 0x14b89635, "arm64_const_caps_ready" },
	{ 0xc51afa7f, "usb_free_urb" },
	{ 0x49cd25ed, "alloc_workqueue" },
	{ 0xb4f20b52, "try_module_get" },
	{ 0xc5d61c20, "usb_alloc_urb" },
};

MODULE_INFO(depends, "kvcommon");

MODULE_ALIAS("usb:v0BFDp0004d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0002d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0005d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v0BFDp0003d*dc*dsc*dp*ic*isc*ip*in*");

MODULE_INFO(srcversion, "CE3AE95F4D4633BEDF33596");
