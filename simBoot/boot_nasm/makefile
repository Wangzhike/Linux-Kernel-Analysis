boot: boot.nasm
	nasm -f bin boot.nasm

.PHONY: run
run:
	qemu-system-x86_64 boot

.PHONY: clean
clean:
	rm -f boot

.PHONY: help
help:
	@echo 'make/make all	编译代码，生成模拟MBR中的boot镜像'
	@echo 'make clean	清除生成的boot镜像'
	@echo 'make run	利用qemu模拟运行由MBR调入内存的boot镜像'
