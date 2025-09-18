flash:
	openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program cmake-build-debug/eq3_rp2040.elf verify reset exit"
