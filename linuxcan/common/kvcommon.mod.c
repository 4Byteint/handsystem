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
	{ 0x59780566, "cdev_del" },
	{ 0x891bb1b4, "kmalloc_caches" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0x6dd185e1, "cdev_init" },
	{ 0xb5b54b34, "_raw_spin_unlock" },
	{ 0x37110088, "remove_wait_queue" },
	{ 0x12a4e128, "__arch_copy_from_user" },
	{ 0xc3690fc, "_raw_spin_lock_bh" },
	{ 0x56470118, "__warn_printk" },
	{ 0xf293ba72, "device_destroy" },
	{ 0x6091b333, "unregister_chrdev_region" },
	{ 0xa648e561, "__ubsan_handle_shift_out_of_bounds" },
	{ 0xd697e69a, "trace_hardirqs_on" },
	{ 0xfb384d37, "kasprintf" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0x25974000, "wait_for_completion" },
	{ 0xdcb764ad, "memset" },
	{ 0x4b0a3f52, "gic_nonsecure_priorities" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x9ec6ca96, "ktime_get_real_ts64" },
	{ 0x9166fada, "strncpy" },
	{ 0x8c03d20c, "destroy_workqueue" },
	{ 0xc1c45ef8, "device_create" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x67182671, "cdev_add" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x6cbbfc54, "__arch_copy_to_user" },
	{ 0xe46021ca, "_raw_spin_unlock_bh" },
	{ 0x8da6585d, "__stack_chk_fail" },
	{ 0x1000e51, "schedule" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0xb8b9f817, "kmalloc_order_trace" },
	{ 0x92997ed8, "_printk" },
	{ 0x908e5601, "cpu_hwcaps" },
	{ 0x69f38847, "cpu_hwcap_keys" },
	{ 0xac1f3f82, "kmem_cache_alloc_trace" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0x4afb2238, "add_wait_queue" },
	{ 0x37a0cba, "kfree" },
	{ 0x4829a47e, "memcpy" },
	{ 0xebf9f00, "class_destroy" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x608741b5, "__init_swait_queue_head" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0xa6257a2f, "complete" },
	{ 0x7f02188f, "__msecs_to_jiffies" },
	{ 0xb788fb30, "gic_pmr_sync" },
	{ 0xec3d2e1b, "trace_hardirqs_off" },
	{ 0xedb10535, "__class_create" },
	{ 0x14b89635, "arm64_const_caps_ready" },
	{ 0x49cd25ed, "alloc_workqueue" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0xe3ec2f2b, "alloc_chrdev_region" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "C83F17716F4E29A0D587420");
