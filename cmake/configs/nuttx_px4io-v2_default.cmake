include(nuttx/px4_impl_nuttx)

px4_nuttx_configure(HWCLASS m3 CONFIG nsh)

set(config_module_list
	modules/px4iofirmware
)
