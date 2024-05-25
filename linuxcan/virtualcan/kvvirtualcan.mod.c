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
	{ 0x891bb1b4, "kmalloc_caches" },
	{ 0x718764f0, "vCanCleanup" },
	{ 0xb5b54b34, "_raw_spin_unlock" },
	{ 0x4d9d0b98, "vCanTime" },
	{ 0x5a8d6745, "vCanRemoveCardChannel" },
	{ 0x679e43d1, "queue_empty" },
	{ 0xdcb764ad, "memset" },
	{ 0xfaa20ff6, "queue_front" },
	{ 0xa14430dc, "vCanDispatchEvent" },
	{ 0xb77cf577, "set_capability_value" },
	{ 0xe6cf5658, "queue_wakeup_on_space" },
	{ 0x220f6eb0, "queue_pop" },
	{ 0x87a21cb3, "__ubsan_handle_out_of_bounds" },
	{ 0x406f7e3f, "vCanInitData" },
	{ 0xa09c6340, "vCanGetCardInfo2" },
	{ 0x8da6585d, "__stack_chk_fail" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0xb8b9f817, "kmalloc_order_trace" },
	{ 0x92997ed8, "_printk" },
	{ 0x69f38847, "cpu_hwcap_keys" },
	{ 0xac1f3f82, "kmem_cache_alloc_trace" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0x50ac5583, "vCanGetCardInfo" },
	{ 0x37a0cba, "kfree" },
	{ 0x4829a47e, "memcpy" },
	{ 0x25418f61, "vCanInit" },
	{ 0x7747eb49, "vCanFlushSendBuffer" },
	{ 0x30372d96, "queue_release" },
	{ 0x75c55d35, "vCanAddCardChannel" },
	{ 0x14b89635, "arm64_const_caps_ready" },
};

MODULE_INFO(depends, "kvcommon");


MODULE_INFO(srcversion, "F7FA974643FF34FEB47B347");
